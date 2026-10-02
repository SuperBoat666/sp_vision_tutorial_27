# 项目理解报告

请尽量使用自己的语言回答以下问题。可以引用少量关键代码或伪代码，但不要只粘贴实现。
完成一节后删除该节末尾的待填写标记；本地检查会拒绝仍有未完成章节的报告。

## 1. 图像生命周期与所有权

解释本项目中图像源为什么会复用缓冲区，以及 `cv::Mat` 的普通复制对底层像素数据
意味着什么。说明你的修改让一个 `Frame` 在进入队列后拥有什么，并解释为何后续读取
不会再改变它。
    图像源复用一个 cv::Mat buffer_ 成员 每次调用 next() 时 它先把新图像读进 raw 再 raw.copyTo(buffer_) 覆盖这块复用的缓冲区模拟相机内部只有一块 buffer 的场景。原来cv::Mat 是浅拷贝，他们的底层像素数据是共享的 所以两个 cv::Mat 会指向同一块像素内存。在我的修改之后改成clone是深拷贝，像素数据是独立出来的，所以内存不与其他共享，因此不会改变

## 2. 并发处理与恰好一次

结合 `BlockingQueue` 的 `push`、`pop` 和 `close` 行为，解释多个 worker 如何分工。
为什么你的实现既不会漏掉已经入队的帧，也不会重复处理同一帧？输入耗尽时，正在等待
以及仍在处理数据的 worker 分别会怎样？
    因为BlockingQueue 内部有一把锁。多个 worker 同时调 pop 时，锁保证同一时刻只有一个 worker 能拿走队首那一帧，拿走后就从队列里删掉了，所以不会重复处理。close只是一个关闭标记，不会清空队列，所以只要队列里还有帧，worker 就会继续取，不会漏帧。当输入耗尽时：producer 调 close叫醒所有 worker。正在等的发现队列空且已关闭，直接退出；还在处理当前帧的，处理完回到 pop，同样发现空且已关闭，最终会退出.

## 3. 共享统计数据

指出哪些线程会读写 `Statistics`。解释原实现中的竞争为什么可能导致错误结果，并说明
你的同步方案提供了什么保证。还应说明取得快照时为什么是安全的。
    Statistics 被以下三类线程访问：1.producer 线程：每产生一帧调用一次 onProduced()。2每个 worker 线程：处理成功调用 onProcessed()，保存成功调用 onSaved()。3.main 线程：Pipeline::statistics() 调用 snapshot() 读取快照。
    在原实现中，四个计数器是普通 int，而 deliberatelySlowIncrement 把自增拆成了先读 old——sleep 100us——写 old+1三步。多线程同时执行时，两个线程可能都读到同一个 old 值，再各自写回 old+1，结果两次自增只让计数增加了 1，产生丢失更新的竞争。就和上课ppt所演示的，会出现数据的丢失（有概率成功，但时间需要卡号）
    我的同步方案是在 Statistics 里加一个mutex_，所有onXxx() 和snapshot()都用 std::lock_guard<std::mutex> 保护。这样每个计数器在计算时会受到保护。同时mutex_ 声明为 mutable，是为了让 const 的 snapshot() 也能加锁（加锁会修改 mutex 自身状态，但逻辑上不改变 Statistics 的数据）。保证取得快照是安全的


## 4. 线程关闭协议

分别描述以下两条路径中的事件顺序，并解释为什么不会发生 `std::terminate`、悬空访问
或永久等待：

1. 调用者执行 `start()` 后显式调用 `wait()`；
2. 调用者执行 `start()` 后不调用 `wait()`，直接让 `Pipeline` 析构。

如果你的实现允许某个生命周期方法被重复调用，也请说明其行为；如果不允许，请说明前置条件。
    如果走第1条路径，start会创建 至少 2 个worker 线程和 producer 线程，保存在 workers_ 和 producer_ 中。然后producer 循环调用 source_->next(frame) 读图，成功则 queue_.push(frame) 并 statistics_.onProduced()。接着wait() 依次对 producer_ 和每个 worker 调用 join，阻塞直到全部线程结束。而wait返回时所有线程已回收，不会 std::terminate（因为 std::thread 析构时已 join），不会悬空访问（线程访问的成员都还活着），不会永久等待
    如果走第2条路径先start 后不 wait，直接析构（Pipeline::~Pipeline被调用。析构函数会首先调用 queue_.close()，唤醒所有阻塞在 pop 上的 worker，让它们能够退出；同时阻止 producer 继续往队列里 push（push 里检测到 closed_ 会直接返回）。接着析构函数对 producer_ 和每个 worker 检查 joinable 并调用 join，等所有线程结束后，析构函数返回。
    由于所有线程在析构函数体内已经 join，std::thread 成员析构时不再是 joinable 状态，不会触发 std::terminate。线程运行时访问的 queue_、statistics_、processor_、source_ 都在成员析构之前还活着，没有悬空访问。queue_.close() 保证 worker 不会永久阻塞在 pop 上，所以 join 不会永久等待。
    我的实现中wait可以重复调用，而start不行先 wait 再析构：wait 里 join 过，析构时 joinable 返回 false，跳过，安全。
    连续两次 wait：第二次所有线程都已 join，joinable 返回 false，跳过，安全。
    如果start重复调用，会再次创建线程并追加到 workers_ 中，导致语义混乱。前置条件是 start 只能调用一次



