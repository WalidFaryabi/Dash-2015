/*
 * definitionsCompiler.h
 *
 * Created: 02.02.2015 23:22:23
 *  Author: Will
 */ 


#ifndef DEFINITIONSCOMPILER_H_
#define DEFINITIONSCOMPILER_H_

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#define DISABLE   0
#define ENABLE    1
#ifndef __cplusplus
#if !defined(__bool_true_false_are_defined)
#define false     0
#define true      1
#endif
#endif
#define PASS      0
#define FAIL      1
#define LOW       0
#define HIGH      1

typedef unsigned char           Bool; //!< Boolean.
#ifndef __cplusplus
#if !defined(__bool_true_false_are_defined)
typedef unsigned char           bool; //!< Boolean.
#endif
#endif
typedef int8_t                  S8 ;  //!< 8-bit signed integer.
typedef uint8_t                 U8 ;  //!< 8-bit unsigned integer.
typedef int16_t                 S16;  //!< 16-bit signed integer.
typedef uint16_t                U16;  //!< 16-bit unsigned integer.
typedef uint16_t                le16_t;
typedef uint16_t                be16_t;
typedef int32_t                 S32;  //!< 32-bit signed integer.
typedef uint32_t                U32;  //!< 32-bit unsigned integer.
typedef uint32_t                le32_t;
typedef uint32_t                be32_t;
typedef int64_t                 S64;  //!< 64-bit signed integer.
typedef uint64_t                U64;  //!< 64-bit unsigned integer.
typedef float                   F32;  //!< 32-bit floating-point number.
typedef double                  F64;  //!< 64-bit floating-point number.
typedef uint32_t                iram_size_t;
//! @}


/*! \name Status Types
 */
//! @{
typedef bool                Status_bool_t;  //!< Boolean status.
typedef U8                  Status_t;       //!< 8-bit-coded status.


/*! \name Bit-Field Handling
 */
//! @{

/*! \brief Reads the bits of a value specified by a given bit-mask.
 *
 * \param value Value to read bits from.
 * \param mask  Bit-mask indicating bits to read.
 *
 * \return Read bits.
 */
#define Rd_bits( value, mask)        ((value) & (mask))

/*! \brief Writes the bits of a C lvalue specified by a given bit-mask.
 *
 * \param lvalue  C lvalue to write bits to.
 * \param mask    Bit-mask indicating bits to write.
 * \param bits    Bits to write.
 *
 * \return Resulting value with written bits.
 */
#define Wr_bits(lvalue, mask, bits)  ((lvalue) = ((lvalue) & ~(mask)) |\
                                                 ((bits  ) &  (mask)))

/*! \brief Tests the bits of a value specified by a given bit-mask.
 *
 * \param value Value of which to test bits.
 * \param mask  Bit-mask indicating bits to test.
 *
 * \return \c 1 if at least one of the tested bits is set, else \c 0.
 */
#define Tst_bits( value, mask)  (Rd_bits(value, mask) != 0)

/*! \brief Clears the bits of a C lvalue specified by a given bit-mask.
 *
 * \param lvalue  C lvalue of which to clear bits.
 * \param mask    Bit-mask indicating bits to clear.
 *
 * \return Resulting value with cleared bits.
 */
#define Clr_bits(lvalue, mask)  ((lvalue) &= ~(mask))

/*! \brief Sets the bits of a C lvalue specified by a given bit-mask.
 *
 * \param lvalue  C lvalue of which to set bits.
 * \param mask    Bit-mask indicating bits to set.
 *
 * \return Resulting value with set bits.
 */
#define Set_bits(lvalue, mask)  ((lvalue) |=  (mask))

/*! \brief Toggles the bits of a C lvalue specified by a given bit-mask.
 *
 * \param lvalue  C lvalue of which to toggle bits.
 * \param mask    Bit-mask indicating bits to toggle.
 *
 * \return Resulting value with toggled bits.
 */
#define Tgl_bits(lvalue, mask)  ((lvalue) ^=  (mask))

/*! \brief Reads the bit-field of a value specified by a given bit-mask.
 *
 * \param value Value to read a bit-field from.
 * \param mask  Bit-mask indicating the bit-field to read.
 *
 * \return Read bit-field.
 */
#define Rd_bitfield( value, mask)           (Rd_bits( value, mask) >> ctz(mask))

/*! \brief Writes the bit-field of a C lvalue specified by a given bit-mask.
 *
 * \param lvalue    C lvalue to write a bit-field to.
 * \param mask      Bit-mask indicating the bit-field to write.
 * \param bitfield  Bit-field to write.
 *
 * \return Resulting value with written bit-field.
 */
#define Wr_bitfield(lvalue, mask, bitfield) (Wr_bits(lvalue, mask, (U32)(bitfield) << ctz(mask)))

//! @}
#if (defined __GNUC__) || (defined __CC_ARM)
#   define clz(u)              __builtin_clz(u)
#elif (defined __ICCARM__)
#   define clz(u)              __CLZ(u)
#else
#   define clz(u)              (((u) == 0)          ? 32 : \
                                ((u) & (1ul << 31)) ?  0 : \
                                ((u) & (1ul << 30)) ?  1 : \
                                ((u) & (1ul << 29)) ?  2 : \
                                ((u) & (1ul << 28)) ?  3 : \
                                ((u) & (1ul << 27)) ?  4 : \
                                ((u) & (1ul << 26)) ?  5 : \
                                ((u) & (1ul << 25)) ?  6 : \
                                ((u) & (1ul << 24)) ?  7 : \
                                ((u) & (1ul << 23)) ?  8 : \
                                ((u) & (1ul << 22)) ?  9 : \
                                ((u) & (1ul << 21)) ? 10 : \
                                ((u) & (1ul << 20)) ? 11 : \
                                ((u) & (1ul << 19)) ? 12 : \
                                ((u) & (1ul << 18)) ? 13 : \
                                ((u) & (1ul << 17)) ? 14 : \
                                ((u) & (1ul << 16)) ? 15 : \
                                ((u) & (1ul << 15)) ? 16 : \
                                ((u) & (1ul << 14)) ? 17 : \
                                ((u) & (1ul << 13)) ? 18 : \
                                ((u) & (1ul << 12)) ? 19 : \
                                ((u) & (1ul << 11)) ? 20 : \
                                ((u) & (1ul << 10)) ? 21 : \
                                ((u) & (1ul <<  9)) ? 22 : \
                                ((u) & (1ul <<  8)) ? 23 : \
                                ((u) & (1ul <<  7)) ? 24 : \
                                ((u) & (1ul <<  6)) ? 25 : \
                                ((u) & (1ul <<  5)) ? 26 : \
                                ((u) & (1ul <<  4)) ? 27 : \
                                ((u) & (1ul <<  3)) ? 28 : \
                                ((u) & (1ul <<  2)) ? 29 : \
                                ((u) & (1ul <<  1)) ? 30 : \
                                31)
#endif

/*! \brief Counts the trailing zero bits of the given value considered as a 32-bit integer.
 *
 * \param u Value of which to count the trailing zero bits.
 *
 * \return The count of trailing zero bits in \a u.
 */
#if (defined __GNUC__) || (defined __CC_ARM)
#   define ctz(u)              __builtin_ctz(u)
#else
#   define ctz(u)              ((u) & (1ul <<  0) ?  0 : \
                                (u) & (1ul <<  1) ?  1 : \
                                (u) & (1ul <<  2) ?  2 : \
                                (u) & (1ul <<  3) ?  3 : \
                                (u) & (1ul <<  4) ?  4 : \
                                (u) & (1ul <<  5) ?  5 : \
                                (u) & (1ul <<  6) ?  6 : \
                                (u) & (1ul <<  7) ?  7 : \
                                (u) & (1ul <<  8) ?  8 : \
                                (u) & (1ul <<  9) ?  9 : \
                                (u) & (1ul << 10) ? 10 : \
                                (u) & (1ul << 11) ? 11 : \
                                (u) & (1ul << 12) ? 12 : \
                                (u) & (1ul << 13) ? 13 : \
                                (u) & (1ul << 14) ? 14 : \
                                (u) & (1ul << 15) ? 15 : \
                                (u) & (1ul << 16) ? 16 : \
                                (u) & (1ul << 17) ? 17 : \
                                (u) & (1ul << 18) ? 18 : \
                                (u) & (1ul << 19) ? 19 : \
                                (u) & (1ul << 20) ? 20 : \
                                (u) & (1ul << 21) ? 21 : \
                                (u) & (1ul << 22) ? 22 : \
                                (u) & (1ul << 23) ? 23 : \
                                (u) & (1ul << 24) ? 24 : \
                                (u) & (1ul << 25) ? 25 : \
                                (u) & (1ul << 26) ? 26 : \
                                (u) & (1ul << 27) ? 27 : \
                                (u) & (1ul << 28) ? 28 : \
                                (u) & (1ul << 29) ? 29 : \
                                (u) & (1ul << 30) ? 30 : \
                                (u) & (1ul << 31) ? 31 : \
                                32)
#endif
/**
 * \brief Emit the compiler pragma \a arg.
 *
 * \param arg The pragma directive as it would appear after \e \#pragma
 * (i.e. not stringified).
 */
#define COMPILER_PRAGMA(arg)            _Pragma(#arg)

/**
 * \def COMPILER_PACK_SET(alignment)
 * \brief Set maximum alignment for subsequent struct and union
 * definitions to \a alignment.
 */
#define COMPILER_PACK_SET(alignment)   COMPILER_PRAGMA(pack(alignment))

/**
 * \def COMPILER_PACK_RESET()
 * \brief Set default alignment for subsequent struct and union
 * definitions.
 */
#define COMPILER_PACK_RESET()          COMPILER_PRAGMA(pack())

/**
 * \brief Set aligned boundary.
 */
#if (defined __GNUC__) || (defined __CC_ARM)
#   define COMPILER_ALIGNED(a)    __attribute__((__aligned__(a)))
#elif (defined __ICCARM__)
#   define COMPILER_ALIGNED(a)    COMPILER_PRAGMA(data_alignment = a)
#endif

/**
 * \brief Set word-aligned boundary.
 */
#if (defined __GNUC__) || defined(__CC_ARM)
#define COMPILER_WORD_ALIGNED    __attribute__((__aligned__(4)))
#elif (defined __ICCARM__)
#define COMPILER_WORD_ALIGNED    COMPILER_PRAGMA(data_alignment = 4)
#endif

/**
 * \def __always_inline
 * \brief The function should always be inlined.
 *
 * This annotation instructs the compiler to ignore its inlining
 * heuristics and inline the function no matter how big it thinks it
 * becomes.
 */
#if defined(__CC_ARM)
#   define __always_inline   __forceinline
#elif (defined __GNUC__)
#	define __always_inline   inline __attribute__((__always_inline__))
#elif (defined __ICCARM__)
#	define __always_inline   _Pragma("inline=forced")
#endif

/**
 * \def __no_inline
 * \brief The function should not be inlined.
 *
 * This annotation instructs the compiler to ignore its inlining
 * heuristics and not inline the function.
 */
#if defined(__CC_ARM)
#   define __no_inline   __attribute__((noinline))
#elif (defined __GNUC__)
#	define __no_inline   __attribute__((__noinline__))
#elif (defined __ICCARM__)
#	define __no_inline   _Pragma("inline=never")
#endif

#define Test_align(val, n     ) (!Tst_bits( val, (n) - 1     )   )

/*! \brief Gets alignment of the number \a val with respect to the \a n boundary.
 *
 * \param val Input value.
 * \param n   Boundary.
 *
 * \return Alignment of the number \a val with respect to the \a n boundary.
 */
#define Get_align( val, n     ) (  Rd_bits( val, (n) - 1     )   )

/*! \brief Sets alignment of the lvalue number \a lval to \a alg with respect to the \a n boundary.
 *
 * \param lval  Input/output lvalue.
 * \param n     Boundary.
 * \param alg   Alignment.
 *
 * \return New value of \a lval resulting from its alignment set to \a alg with respect to the \a n boundary.
 */
#define Set_align(lval, n, alg) (  Wr_bits(lval, (n) - 1, alg)   )

/*! \brief Aligns the number \a val with the upper \a n boundary.
 *
 * \param val Input value.
 * \param n   Boundary.
 *
 * \return Value resulting from the number \a val aligned with the upper \a n boundary.
 */
#define Align_up(  val, n     ) (((val) + ((n) - 1)) & ~((n) - 1))

/*! \brief Aligns the number \a val with the lower \a n boundary.
 *
 * \param val Input value.
 * \param n   Boundary.
 *
 * \return Value resulting from the number \a val aligned with the lower \a n boundary.
 */
#define Align_down(val, n     ) ( (val)              & ~((n) - 1))

//! @}


/*! \name Mathematics
 *
 * The same considerations as for clz and ctz apply here but GCC does not
 * provide built-in functions to access the assembly instructions abs, min and
 * max and it does not produce them by itself in most cases, so two sets of
 * macros are defined here:
 *   - Abs, Min and Max to apply to constant expressions (values known at
 *     compile time);
 *   - abs, min and max to apply to non-constant expressions (values unknown at
 *     compile time), abs is found in stdlib.h.
 */
//! @{

/*! \brief Takes the absolute value of \a a.
 *
 * \param a Input value.
 *
 * \return Absolute value of \a a.
 *
 * \note More optimized if only used with values known at compile time.
 */
#define Abs(a)              (((a) <  0 ) ? -(a) : (a))

/*! \brief Takes the minimal value of \a a and \a b.
 *
 * \param a Input value.
 * \param b Input value.
 *
 * \return Minimal value of \a a and \a b.
 *
 * \note More optimized if only used with values known at compile time.
 */
#define Min(a, b)           (((a) < (b)) ?  (a) : (b))

/*! \brief Takes the maximal value of \a a and \a b.
 *
 * \param a Input value.
 * \param b Input value.
 *
 * \return Maximal value of \a a and \a b.
 *
 * \note More optimized if only used with values known at compile time.
 */
#define Max(a, b)           (((a) > (b)) ?  (a) : (b))

// abs() is already defined by stdlib.h

/*! \brief Takes the minimal value of \a a and \a b.
 *
 * \param a Input value.
 * \param b Input value.
 *
 * \return Minimal value of \a a and \a b.
 *
 * \note More optimized if only used with values unknown at compile time.
 */
#define min(a, b)   Min(a, b)

/*! \brief Takes the maximal value of \a a and \a b.
 *
 * \param a Input value.
 * \param b Input value.
 *
 * \return Maximal value of \a a and \a b.
 *
 * \note More optimized if only used with values unknown at compile time.
 */
#define max(a, b)   Max(a, b)
#define Long_call(addr)                   ((*(void (*)(void))(addr))())


/*! \name MCU Endianism Handling
 * ARM is MCU little endianism.
 */
//! @{
#define  MSB(u16)       (((U8  *)&(u16))[1]) //!< Most significant byte of \a u16.
#define  LSB(u16)       (((U8  *)&(u16))[0]) //!< Least significant byte of \a u16.

#define  MSH(u32)       (((U16 *)&(u32))[1]) //!< Most significant half-word of \a u32.
#define  LSH(u32)       (((U16 *)&(u32))[0]) //!< Least significant half-word of \a u32.
#define  MSB0W(u32)     (((U8  *)&(u32))[3]) //!< Most significant byte of 1st rank of \a u32.
#define  MSB1W(u32)     (((U8  *)&(u32))[2]) //!< Most significant byte of 2nd rank of \a u32.
#define  MSB2W(u32)     (((U8  *)&(u32))[1]) //!< Most significant byte of 3rd rank of \a u32.
#define  MSB3W(u32)     (((U8  *)&(u32))[0]) //!< Most significant byte of 4th rank of \a u32.
#define  LSB3W(u32)     MSB0W(u32)           //!< Least significant byte of 4th rank of \a u32.
#define  LSB2W(u32)     MSB1W(u32)           //!< Least significant byte of 3rd rank of \a u32.
#define  LSB1W(u32)     MSB2W(u32)           //!< Least significant byte of 2nd rank of \a u32.
#define  LSB0W(u32)     MSB3W(u32)           //!< Least significant byte of 1st rank of \a u32.
        
#define  MSW(u64)       (((U32 *)&(u64))[1]) //!< Most significant word of \a u64.
#define  LSW(u64)       (((U32 *)&(u64))[0]) //!< Least significant word of \a u64.
#define  MSH0(u64)      (((U16 *)&(u64))[3]) //!< Most significant half-word of 1st rank of \a u64.
#define  MSH1(u64)      (((U16 *)&(u64))[2]) //!< Most significant half-word of 2nd rank of \a u64.
#define  MSH2(u64)      (((U16 *)&(u64))[1]) //!< Most significant half-word of 3rd rank of \a u64.
#define  MSH3(u64)      (((U16 *)&(u64))[0]) //!< Most significant half-word of 4th rank of \a u64.
#define  LSH3(u64)      MSH0(u64)            //!< Least significant half-word of 4th rank of \a u64.
#define  LSH2(u64)      MSH1(u64)            //!< Least significant half-word of 3rd rank of \a u64.
#define  LSH1(u64)      MSH2(u64)            //!< Least significant half-word of 2nd rank of \a u64.
#define  LSH0(u64)      MSH3(u64)            //!< Least significant half-word of 1st rank of \a u64.
#define  MSB0D(u64)     (((U8  *)&(u64))[7]) //!< Most significant byte of 1st rank of \a u64.
#define  MSB1D(u64)     (((U8  *)&(u64))[6]) //!< Most significant byte of 2nd rank of \a u64.
#define  MSB2D(u64)     (((U8  *)&(u64))[5]) //!< Most significant byte of 3rd rank of \a u64.
#define  MSB3D(u64)     (((U8  *)&(u64))[4]) //!< Most significant byte of 4th rank of \a u64.
#define  MSB4D(u64)     (((U8  *)&(u64))[3]) //!< Most significant byte of 5th rank of \a u64.
#define  MSB5D(u64)     (((U8  *)&(u64))[2]) //!< Most significant byte of 6th rank of \a u64.
#define  MSB6D(u64)     (((U8  *)&(u64))[1]) //!< Most significant byte of 7th rank of \a u64.
#define  MSB7D(u64)     (((U8  *)&(u64))[0]) //!< Most significant byte of 8th rank of \a u64.
#define  LSB7D(u64)     MSB0D(u64)           //!< Least significant byte of 8th rank of \a u64.
#define  LSB6D(u64)     MSB1D(u64)           //!< Least significant byte of 7th rank of \a u64.
#define  LSB5D(u64)     MSB2D(u64)           //!< Least significant byte of 6th rank of \a u64.
#define  LSB4D(u64)     MSB3D(u64)           //!< Least significant byte of 5th rank of \a u64.
#define  LSB3D(u64)     MSB4D(u64)           //!< Least significant byte of 4th rank of \a u64.
#define  LSB2D(u64)     MSB5D(u64)           //!< Least significant byte of 3rd rank of \a u64.
#define  LSB1D(u64)     MSB6D(u64)           //!< Least significant byte of 2nd rank of \a u64.
#define  LSB0D(u64)     MSB7D(u64)           //!< Least significant byte of 1st rank of \a u64.

#define  BE16(x)        Swap16(x)
#define  LE16(x)        (x)

#define  le16_to_cpu(x) (x)
#define  cpu_to_le16(x) (x)
#define  LE16_TO_CPU(x) (x)
#define  CPU_TO_LE16(x) (x)

#define  be16_to_cpu(x) Swap16(x)
#define  cpu_to_be16(x) Swap16(x)
#define  BE16_TO_CPU(x) Swap16(x)
#define  CPU_TO_BE16(x) Swap16(x)

#define  le32_to_cpu(x) (x)
#define  cpu_to_le32(x) (x)
#define  LE32_TO_CPU(x) (x)
#define  CPU_TO_LE32(x) (x)

#define  be32_to_cpu(x) swap32(x)
#define  cpu_to_be32(x) swap32(x)
#define  BE32_TO_CPU(x) swap32(x)
#define  CPU_TO_BE32(x) swap32(x)
//! @}

/*! \name Endianism Conversion
 *
 * The same considerations as for clz and ctz apply here but GCC's
 * __builtin_bswap_32 and __builtin_bswap_64 do not behave like macros when
 * applied to constant expressions, so two sets of macros are defined here:
 *   - Swap16, Swap32 and Swap64 to apply to constant expressions (values known
 *     at compile time);
 *   - swap16, swap32 and swap64 to apply to non-constant expressions (values
 *     unknown at compile time).
 */
//! @{

/*! \brief Toggles the endianism of \a u16 (by swapping its bytes).
 *
 * \param u16 U16 of which to toggle the endianism.
 *
 * \return Value resulting from \a u16 with toggled endianism.
 *
 * \note More optimized if only used with values known at compile time.
 */
#define Swap16(u16) ((U16)(((U16)(u16) >> 8) |\
                           ((U16)(u16) << 8)))

/*! \brief Toggles the endianism of \a u32 (by swapping its bytes).
 *
 * \param u32 U32 of which to toggle the endianism.
 *
 * \return Value resulting from \a u32 with toggled endianism.
 *
 * \note More optimized if only used with values known at compile time.
 */
#define Swap32(u32) ((U32)(((U32)Swap16((U32)(u32) >> 16)) |\
                           ((U32)Swap16((U32)(u32)) << 16)))

/*! \brief Toggles the endianism of \a u64 (by swapping its bytes).
 *
 * \param u64 U64 of which to toggle the endianism.
 *
 * \return Value resulting from \a u64 with toggled endianism.
 *
 * \note More optimized if only used with values known at compile time.
 */
#define Swap64(u64) ((U64)(((U64)Swap32((U64)(u64) >> 32)) |\
                           ((U64)Swap32((U64)(u64)) << 32)))

/*! \brief Toggles the endianism of \a u16 (by swapping its bytes).
 *
 * \param u16 U16 of which to toggle the endianism.
 *
 * \return Value resulting from \a u16 with toggled endianism.
 *
 * \note More optimized if only used with values unknown at compile time.
 */
#define swap16(u16) Swap16(u16)

/*! \brief Toggles the endianism of \a u32 (by swapping its bytes).
 *
 * \param u32 U32 of which to toggle the endianism.
 *
 * \return Value resulting from \a u32 with toggled endianism.
 *
 * \note More optimized if only used with values unknown at compile time.
 */
#if (defined __GNUC__)
#   define swap32(u32) ((U32)__builtin_bswap32((U32)(u32)))
#else
#   define swap32(u32) Swap32(u32)
#endif

/*! \brief Toggles the endianism of \a u64 (by swapping its bytes).
 *
 * \param u64 U64 of which to toggle the endianism.
 *
 * \return Value resulting from \a u64 with toggled endianism.
 *
 * \note More optimized if only used with values unknown at compile time.
 */
#if (defined __GNUC__)
#   define swap64(u64) ((U64)__builtin_bswap64((U64)(u64)))
#else
#   define swap64(u64) ((U64)(((U64)swap32((U64)(u64) >> 32)) |\
                           ((U64)swap32((U64)(u64)) << 32)))
#endif


#define LSB0(u32)           LSB0W(u32)  //!< Least significant byte of 1st rank of \a u32.
#define LSB1(u32)           LSB1W(u32)  //!< Least significant byte of 2nd rank of \a u32.
#define LSB2(u32)           LSB2W(u32)  //!< Least significant byte of 3rd rank of \a u32.
#define LSB3(u32)           LSB3W(u32)  //!< Least significant byte of 4th rank of \a u32.
#define MSB3(u32)           MSB3W(u32)  //!< Most significant byte of 4th rank of \a u32.
#define MSB2(u32)           MSB2W(u32)  //!< Most significant byte of 3rd rank of \a u32.
#define MSB1(u32)           MSB1W(u32)  //!< Most significant byte of 2nd rank of \a u32.
#define MSB0(u32)           MSB0W(u32)  //!< Most significant byte of 1st rank of \a u32.


/**
 * \def UNUSED
 * \brief Marking \a v as a unused parameter or value.
 */
#define UNUSED(v)          (void)(v)

/**
 * \def unused
 * \brief Marking \a v as a unused parameter or value.
 */
#define unused(v)          do { (void)(v); } while(0)

#endif /* DEFINITIONSCOMPILER_H_ */