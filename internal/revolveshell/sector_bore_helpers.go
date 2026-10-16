package revolveshell

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

type SectorBandCross struct {
	X, Round, SlopeLo, SlopeHi float64
}

// RevolveSectorStraightBandCross solves a circle/straight-meridian crossing
// with exact dyadic source coordinates. Its rounded axial mesh coordinate
// carries a separate outward error and slope interval.
func RevolveSectorStraightBandCross(outer, capPoint sectionrecord.Point2, radius float64) (SectorBandCross, bool) {
	if !proofbound.FiniteVec(r3.Vec{X: outer.U, Y: outer.V, Z: radius}) ||
		!proofbound.FiniteVec(r3.Vec{X: capPoint.U, Y: capPoint.V}) {
		return SectorBandCross{}, false
	}
	outerX, outerR := proofarith.FloatRat(outer.U), proofarith.FloatRat(outer.V)
	capX, capR := proofarith.FloatRat(capPoint.U), proofarith.FloatRat(capPoint.V)
	rad := proofarith.FloatRat(radius)
	dx := new(big.Rat).Sub(capX, outerX)
	dr := new(big.Rat).Sub(outerR, capR)
	if dx.Sign() <= 0 || dr.Sign() <= 0 || rad.Cmp(capR) <= 0 || rad.Cmp(outerR) >= 0 {
		return SectorBandCross{}, false
	}
	fraction := new(big.Rat).Quo(new(big.Rat).Sub(outerR, rad), dr)
	crossRat := new(big.Rat).Add(outerX, new(big.Rat).Mul(dx, fraction))
	if crossRat.Cmp(outerX) <= 0 || crossRat.Cmp(capX) >= 0 {
		return SectorBandCross{}, false
	}
	crossX, _ := crossRat.Float64()
	crossGap := new(big.Rat).Sub(crossRat, proofarith.FloatRat(crossX))
	crossGap.Abs(crossGap)
	slope := new(big.Rat).Quo(dr, dx)
	cross := SectorBandCross{X: crossX, Round: proofbound.RatFloatUp(crossGap),
		SlopeLo: proofbound.RatFloatDown(slope), SlopeHi: proofbound.RatFloatUp(slope)}
	if !proofbound.FiniteVec(r3.Vec{X: cross.X, Y: cross.SlopeLo, Z: cross.SlopeHi}) ||
		cross.SlopeLo <= 0 || cross.Round < 0 {
		return SectorBandCross{}, false
	}
	return cross, true
}

func RevolveSectorSortedTriangle(t [3]int) [3]int {
	if t[0] > t[1] {
		t[0], t[1] = t[1], t[0]
	}
	if t[1] > t[2] {
		t[1], t[2] = t[2], t[1]
	}
	if t[0] > t[1] {
		t[0], t[1] = t[1], t[0]
	}
	return t
}

func RevolveSectorTriangleClear(t [3]int, verts []r3.Vec, clearance float64) bool {
	a, b, c := verts[t[0]], verts[t[1]], verts[t[2]]
	loY, hiY := min(a.Y, b.Y, c.Y), max(a.Y, b.Y, c.Y)
	loZ, hiZ := min(a.Z, b.Z, c.Z), max(a.Z, b.Z, c.Z)
	if loY > clearance || hiY < -clearance || loZ > clearance || hiZ < -clearance {
		return true
	}
	projected := [3]r3.Vec{{X: a.Y, Y: a.Z}, {X: b.Y, Y: b.Z}, {X: c.Y, Y: c.Z}}
	return loftcylinder.TriangleMinRadius(projected[:], [3]int{0, 1, 2}) > clearance
}
