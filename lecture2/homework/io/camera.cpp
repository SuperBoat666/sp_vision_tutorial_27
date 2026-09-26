#include "camera.hpp"
#include <unordered_map>
//构造函数
Camera::Camera() {
    MV_CC_DEVICE_INFO_LIST list;
    MV_CC_EnumDevices(MV_USB_DEVICE, &list);

    MV_CC_CreateHandle(&handle_, list.pDeviceInfo[0]);
    MV_CC_OpenDevice(handle_);

    MV_CC_SetEnumValue(handle_, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
    MV_CC_SetEnumValue(handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
    MV_CC_SetEnumValue(handle_, "GainAuto", MV_GAIN_MODE_OFF);
    MV_CC_SetFloatValue(handle_, "ExposureTime", 10000);
    MV_CC_SetFloatValue(handle_, "Gain", 16.9);
    MV_CC_SetFrameRate(handle_, 60);

    MV_CC_StartGrabbing(handle_);
}
//析构函数
Camera::~Camera() {
    MV_CC_StopGrabbing(handle_);
    MV_CC_CloseDevice(handle_);
    MV_CC_DestroyHandle(handle_);
}

bool Camera::read(cv::Mat& img) {
    // 1. 取一帧，最多等 100ms
    MV_FRAME_OUT raw;
    if (MV_CC_GetImageBuffer(handle_, &raw, 100) != MV_OK) return false;

    // 2. 用原始数据构造单通道 Mat
    MV_CC_PIXEL_CONVERT_PARAM cvt{};
    img = cv::Mat(raw.stFrameInfo.nHeight, raw.stFrameInfo.nWidth, CV_8U, raw.pBufAddr);

    // 3. 填充转换参数
    cvt.nWidth = raw.stFrameInfo.nWidth;
    cvt.nHeight = raw.stFrameInfo.nHeight;
    cvt.pSrcData = raw.pBufAddr;
    cvt.nSrcDataLen = raw.stFrameInfo.nFrameLen;
    cvt.enSrcPixelType = raw.stFrameInfo.enPixelType;
    cvt.pDstBuffer = img.data;
    cvt.nDstBufferSize = img.total() * img.elemSize();
    cvt.enDstPixelType = PixelType_Gvsp_BGR8_Packed;

    // 4. Bayer → BGR
    static const std::unordered_map<MvGvspPixelType, cv::ColorConversionCodes> type_map = {
        {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2RGB},
        {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2RGB},
        {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2RGB},
        {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2RGB}};
    cv::cvtColor(img, img, type_map.at(raw.stFrameInfo.enPixelType));

    // 5. 释放缓冲区
    MV_CC_FreeImageBuffer(handle_, &raw);
    return true;
}