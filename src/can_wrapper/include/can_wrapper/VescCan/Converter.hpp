#ifndef VescCan_Converter_h_
#define VescCan_Converter_h_

#include <stdint.h>
#include <type_traits>
#include <boost/endian/conversion.hpp>
#include <boost/endian/arithmetic.hpp>

namespace VescCan
{

    template< typename NativeType, class Buffer, int32_t scale >
    class Converter
    {
    public:
        static Buffer encode(NativeType value)
        {
            value *= scale;
            return Buffer(value);
        }
        static NativeType decode(Buffer buffer)
        {
            NativeType value = buffer.value();
            return value / scale;
        }
    };

    template< class Buffer, int32_t scale >
    using ConverterI = Converter<int,Buffer,scale>;

    template< int32_t scale >
    using ConverterI8 = ConverterI<boost::endian::big_int8_buf_t,scale>;
    template< int32_t scale >
    using ConverterI16 = ConverterI<boost::endian::big_int16_buf_t,scale>;
    template< int32_t scale >
    using ConverterI32 = ConverterI<boost::endian::big_int32_buf_t,scale>;


    template< class Buffer, int32_t scale >
    using ConverterF = Converter<float,Buffer,scale>;

    template< int32_t scale >
    using ConverterF8 = ConverterF<boost::endian::big_int8_buf_t,scale>;
    template< int32_t scale >
    using ConverterF16 = ConverterF<boost::endian::big_int16_buf_t,scale>;
    template< int32_t scale >
    using ConverterF32 = ConverterF<boost::endian::big_int32_buf_t,scale>;
}

#endif // VescCan_Converter_h_
