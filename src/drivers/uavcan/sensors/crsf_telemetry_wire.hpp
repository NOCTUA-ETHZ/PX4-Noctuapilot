// Copyright (c) 2026 Noctua contributors. SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include <stdint.h>
#include <stddef.h>
namespace noctua_crsf {
static constexpr uint8_t magic[4] = {'N','C','T',1};
inline uint8_t crc(const uint8_t *p, size_t n) {
    uint8_t c=0;
    while(n--) { c^=*p++; for(unsigned i=0;i<8;++i) { c=(c&0x80)?uint8_t((c<<1)^0xd5):uint8_t(c<<1); } }
    return c;
}
inline bool valid(const uint8_t *p, size_t n) {
    if(n<8 || n>68) { return false; }
    for(unsigned i=0;i<4;++i) { if(p[i]!=magic[i]) { return false; } }
    p+=4;n-=4;
    if(p[0]!=0xc8 || p[1]+2u!=n || crc(p+2,n-3)!=p[n-1]) { return false; }
    switch(p[2]) {
    case 0x02: return n==19;
    case 0x08: return n==12;
    case 0x1e: return n==10;
    case 0x21: return n>=5 && n<=20 && p[n-2]==0;
    default: return false;
    }
}
struct Frame {
    uint8_t bytes[68]{};
    unsigned size{7};
    explicit Frame(uint8_t type) {
        for(unsigned i=0;i<4;++i) { bytes[i]=magic[i]; }
        bytes[4]=0xc8;bytes[6]=type;
    }
    void put(uint32_t v,unsigned w) { while(w && size<sizeof(bytes)-1) { bytes[size++]=uint8_t(v>>(8*--w)); } }
    void finish() { bytes[5]=size-5; const uint8_t c=crc(bytes+6,size-6);bytes[size++]=c; }
};
}
