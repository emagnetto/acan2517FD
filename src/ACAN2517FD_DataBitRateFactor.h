//------------------------------------------------------------------------------
// A CANFD driver
// by Pierre Molinaro

// This header is common to libraries
//        https://github.com/pierremolinaro/ACAN_T4
//        https://github.com/pierremolinaro/ACAN2517FD
//
//------------------------------------------------------------------------------

#ifndef ACANFD_DATA_BIT_RATE_FACTOR_DEFINED
#define ACANFD_DATA_BIT_RATE_FACTOR_DEFINED

//------------------------------------------------------------------------------

#include <stdint.h>

//------------------------------------------------------------------------------

/// @brief Valores de arbitración en kb/s
enum class ArbitrationBitRateValue : uint32_t {
    v125 = 125000,
   // v250 = 250000,
    v500 = 500000,  //Default, según DT3CanConnect -> GetEEPROM_CanData_From_Device
    //v750 = 750000,
    v1000 = 1000000
} ;

constexpr ArbitrationBitRateValue ArbitrationBitRatesArr[] = {
    ArbitrationBitRateValue::v125,
    //ArbitrationBitRateValue::v250,
    ArbitrationBitRateValue::v500, 
    //ArbitrationBitRateValue::v750,
    ArbitrationBitRateValue::v1000
};

constexpr uint8_t DefaultArbitrationBitRateIndex = 1u;

constexpr uint8_t ArbitrationBitRateArrCount =
    sizeof(ArbitrationBitRatesArr) / sizeof(ArbitrationBitRatesArr[0]);

enum class DataBitRateFactor : uint8_t {
    x1 = 1, //Default, según DT3CanConnect -> GetEEPROM_CanData_From_Device
    x2 = 2, 
   // x3 = 3,
    x4 = 4,
    x5 = 5,
   // x6 = 6,
  //  x7 = 7,
    x8 = 8,
//    x9 = 9,
 //   x10 = 10
} ;


constexpr DataBitRateFactor DataBitRateFactorArr[] = {
    DataBitRateFactor::x1,
    DataBitRateFactor::x2,  
   // DataBitRateFactor::x3,
    DataBitRateFactor::x4,
    DataBitRateFactor::x5,
  //  DataBitRateFactor::x6,
 //   DataBitRateFactor::x7,
    DataBitRateFactor::x8,
 //   DataBitRateFactor::x9,
 //   DataBitRateFactor::x10
};

constexpr uint8_t DefaultDataBitRateFactorIndex = 0u;

constexpr uint8_t DataBitRateFactorArrCount =
    sizeof(DataBitRateFactorArr) / sizeof(DataBitRateFactorArr[0]);
    
//------------------------------------------------------------------------------

#endif
