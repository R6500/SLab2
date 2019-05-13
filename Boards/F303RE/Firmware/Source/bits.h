/**************************************
  bits.h
  Bit manipulation

  History:
    22/05/2018 : First version

***************************************/

#ifndef BITS_MODULE
#define BITS_MODULE

#define BIT_SET(var,bit)               (var) |= (bit)
#define BIT_CLEAR(var,bit)             (var) &= (~(bit))
#define BIT_FIELD_WRITE(var,pos,mask,value)  (var) = (((var)&(~((mask)<<(pos))))|((value)<<(pos)))
#define BIT_FIELD_READ(var,pos,mask)   (((var)>>(pos))&(mask))

#endif  //BITS_MODULE
