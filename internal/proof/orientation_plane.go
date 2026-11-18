package proof

import "github.com/lestrrat-3d/r3"

// FloatOrientationPlane caches the triangle-only terms of OrientSignFloat.
// Its immutable values can be shared across concurrent point queries.
type FloatOrientationPlane struct {
	anchor, normal, magnitude r3.Vec
	zeroAxes                  uint8
	stable                    bool
}

// FloatOrientationPlaneWithOriginalAssociation caches the triangle terms
// needed to preserve OrientSignFloat's determinant grouping.
type FloatOrientationPlaneWithOriginalAssociation struct {
	anchor       r3.Vec
	ba, ca       r3.Vec
	absBA, absCA r3.Vec
	zeroAxes     uint8
	stable       bool
}

// NewFloatOrientationPlane prepares the oriented plane through a, b and c.
// All input coordinates must be finite, as for OrientSignFloat.
func NewFloatOrientationPlane(a, b, c r3.Vec) FloatOrientationPlane {
	ba, ca := b.Sub(a), c.Sub(a)
	zeroAxes, stable := floatOrientationPlaneGuard(ba, ca)
	p := FloatOrientationPlane{anchor: a, zeroAxes: zeroAxes, stable: stable}
	if !stable {
		return p
	}
	p.normal = ba.Cross(ca)
	p.magnitude = r3.NewVec(
		Abs(ba.Y)*Abs(ca.Z)+Abs(ba.Z)*Abs(ca.Y),
		Abs(ba.Z)*Abs(ca.X)+Abs(ba.X)*Abs(ca.Z),
		Abs(ba.X)*Abs(ca.Y)+Abs(ba.Y)*Abs(ca.X))
	return p
}

func floatOrientationPlaneGuard(ba, ca r3.Vec) (uint8, bool) {
	var zeroAxes uint8
	stable := true
	for axis, pair := range [...][2]float64{{ba.X, ca.X}, {ba.Y, ca.Y}, {ba.Z, ca.Z}} {
		if pair == [2]float64{} {
			zeroAxes |= 1 << axis
		}
		for _, v := range pair {
			x := Abs(v)
			if !(x == 0 || x >= 0x1p-300 && x <= 0x1p250) {
				stable = false
			}
		}
	}
	return zeroAxes, stable
}

// NewFloatOrientationPlaneWithOriginalAssociation prepares a triangle for
// repeated probes while preserving OrientSignFloat's determinant grouping.
// All input coordinates must be finite, as for OrientSignFloat.
func NewFloatOrientationPlaneWithOriginalAssociation(a, b, c r3.Vec) FloatOrientationPlaneWithOriginalAssociation {
	ba, ca := b.Sub(a), c.Sub(a)
	zeroAxes, stable := floatOrientationPlaneGuard(ba, ca)
	return FloatOrientationPlaneWithOriginalAssociation{
		anchor: a, ba: ba, ca: ca,
		absBA:    r3.NewVec(Abs(ba.X), Abs(ba.Y), Abs(ba.Z)),
		absCA:    r3.NewVec(Abs(ca.X), Abs(ca.Y), Abs(ca.Z)),
		zeroAxes: zeroAxes, stable: stable,
	}
}

// SignWithOriginalAssociation applies OrientSignFloat's determinant and
// permanent expression to a prepared plane. It preserves that expression's
// grouping, and falls back to OrientSignFloat when the float proof is
// uncertain. b and c are the original triangle corners used to prepare p.
func (p *FloatOrientationPlaneWithOriginalAssociation) SignWithOriginalAssociation(b, c, d r3.Vec) (int, bool) {
	da := d.Sub(p.anchor)
	if p.zeroAxes&1 != 0 && da.X == 0 || p.zeroAxes&2 != 0 && da.Y == 0 ||
		p.zeroAxes&4 != 0 && da.Z == 0 {
		return 0, true
	}
	if !p.stable {
		return OrientSignFloat(p.anchor, b, c, d)
	}
	dax, day, daz := da.X, da.Y, da.Z
	for _, v := range [...]float64{dax, day, daz} {
		x := Abs(v)
		if !(x == 0 || (x >= 0x1p-300 && x <= 0x1p250)) {
			return OrientSignFloat(p.anchor, b, c, d)
		}
	}
	absDax, absDay, absDaz := Abs(dax), Abs(day), Abs(daz)
	det := p.ba.X*(p.ca.Y*daz-p.ca.Z*day) + p.ba.Y*(p.ca.Z*dax-p.ca.X*daz) +
		p.ba.Z*(p.ca.X*day-p.ca.Y*dax)
	perm := p.absBA.X*(p.absCA.Y*absDaz+p.absCA.Z*absDay) +
		p.absBA.Y*(p.absCA.Z*absDax+p.absCA.X*absDaz) +
		p.absBA.Z*(p.absCA.X*absDay+p.absCA.Y*absDax)
	err := 2e-15 * perm
	if det > err {
		return 1, true
	}
	if det < -err {
		return -1, true
	}
	return OrientSignFloat(p.anchor, b, c, d)
}

// Sign returns an exact orientation sign when its float error bound decides
// it. An uncertain result must use exact arithmetic. This factors the same
// six triple products as OrientSignFloat, grouping by the probe differences.
// Each term still has at most eight roundings. Its guard and 2e-15 permanent
// bound retain the normal-range and subnormal-grid proof in OrientSignFloat.
func (p *FloatOrientationPlane) Sign(d r3.Vec) (int, bool) {
	da := d.Sub(p.anchor)
	if p.zeroAxes&1 != 0 && da.X == 0 || p.zeroAxes&2 != 0 && da.Y == 0 ||
		p.zeroAxes&4 != 0 && da.Z == 0 {
		return 0, true
	}
	if !p.stable {
		return 0, false
	}
	for _, v := range [...]float64{da.X, da.Y, da.Z} {
		x := Abs(v)
		if !(x == 0 || x >= 0x1p-300 && x <= 0x1p250) {
			return 0, false
		}
	}
	det := p.normal.Dot(da)
	perm := p.magnitude.Dot(r3.NewVec(Abs(da.X), Abs(da.Y), Abs(da.Z)))
	err := 2e-15 * perm
	if det > err {
		return 1, true
	}
	if det < -err {
		return -1, true
	}
	return 0, false
}
