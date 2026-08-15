#pragma once

enum SPI_Result
{
    SPI_Result_Success = 0,

    // 参数错误
    SPI_Result_InvalidArgs,
    // 失败
    SPI_Result_Fail,

    // 超时
    SPI_Result_timeout,
};