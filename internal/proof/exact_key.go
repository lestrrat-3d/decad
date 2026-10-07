package proof

import (
	"encoding/binary"
	"math/big"
)

// Key is the exact identity of the point: two points weld exactly when their
// CANONICAL homogeneous coordinates are identical — stitching by shared exact
// vertices, never by distance (docs/evaluator-design.md §9). A homogeneous
// point has many spellings, so the raw fields are never compared directly;
// XhpCanon collapses every spelling of one coordinate to one four-tuple
// before the key is built.
func (p Xpt) Key() string {
	c := XhpCanon(Xhp(p))
	return ExactIntsKey(c.X, c.Y, c.Z, c.W)
}

// ExactIntsKey encodes signed integers without decimal conversion. Each value
// carries its sign and byte length, so adjacent magnitudes cannot collide.
func ExactIntsKey(values ...*big.Int) string {
	size := 9 * len(values)
	for _, v := range values {
		size += (v.BitLen() + 7) / 8
	}
	buf := make([]byte, 0, size)
	for _, v := range values {
		buf = append(buf, byte(v.Sign()+1))
		n := (v.BitLen() + 7) / 8
		buf = binary.LittleEndian.AppendUint64(buf, uint64(n))
		start := len(buf)
		buf = buf[:start+n]
		v.FillBytes(buf[start:])
	}
	return string(buf)
}

// DvPrimitive returns the primitive integer vector along a nonzero v: v times
// the positive rational that makes its components integers with no common
// factor. Two nonzero vectors share it exactly when one is a positive multiple
// of the other, so it names v's direction, sign kept. A zero v returns zero.
func DvPrimitive(v DyV3) DyV3 {
	exp, found := 0, false
	for _, c := range v {
		if c.IsZero() {
			continue
		}
		if !found || c.exp < exp {
			exp = c.exp
		}
		found = true
	}
	if !found {
		return DyV3{}
	}
	// The components are named locals, never elements of a local array indexed
	// by a loop variable around a call (dyadic.go, "Indexed reads").
	v0, v1, v2 := v[0], v[1], v[2]
	gcd := new(big.Int)
	scaled := func(c Dyadic) *big.Int {
		if c.IsZero() {
			return nil
		}
		k := c.MantInto(new(big.Int))
		k.Lsh(k, uint(c.exp-exp))
		if gcd.Sign() == 0 {
			gcd.Abs(k)
			return k
		}
		gcd.GCD(nil, nil, gcd, new(big.Int).Abs(k))
		return k
	}
	k0, k1, k2 := scaled(v0), scaled(v1), scaled(v2)
	primitive := func(k *big.Int) Dyadic {
		if k == nil {
			return Dyadic{}
		}
		if gcd.BitLen() > 1 {
			k.Quo(k, gcd)
		}
		return fromBig(k, 0)
	}
	return DyV3{primitive(k0), primitive(k1), primitive(k2)}
}

// AppendKey appends a byte encoding of d to b. Two encodings are equal exactly
// when the values are: the encoding reads the reduced, canonical form, whose
// fields are equal exactly when the values are, and it prefixes the
// mantissa's length so no two encodings run together. The magnitude is
// written big-endian without leading zero bytes, the form big.Int.Bytes
// gives, wherever the mantissa is held.
func (d Dyadic) AppendKey(b []byte) []byte {
	if d.IsZero() {
		return append(b, 0)
	}
	sign := byte(1)
	if d.Sign() < 0 {
		sign = 2
	}
	b = append(b, sign)
	b = binary.AppendVarint(b, int64(d.exp))
	if d.big != nil {
		words := d.big.Bytes()
		b = binary.AppendUvarint(b, uint64(len(words)))
		return append(b, words...)
	}
	n := (bitLen128(d.lo, d.hi&^dySign) + 7) / 8
	b = binary.AppendUvarint(b, uint64(n))
	var bytes [16]byte
	binary.BigEndian.PutUint64(bytes[:8], d.hi&^dySign)
	binary.BigEndian.PutUint64(bytes[8:], d.lo)
	return append(b, bytes[len(bytes)-n:]...)
}
