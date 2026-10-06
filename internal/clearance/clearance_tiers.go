package clearance

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// EdgeWits returns on-edge sample points.
func EdgeWits(e *CEdge) []r3.Vec {
	if e.Line {
		return []r3.Vec{e.A, e.B, e.A.Add(e.B).Scale(0.5)}
	}
	lo, hi := 0.0, 2*math.Pi
	if !e.Ang.Full {
		lo, hi = e.Ang.Lo, e.Ang.Hi
	}
	return []r3.Vec{e.At(lo), e.At((lo + hi) / 2)}
}

// LineParamAdmit classifies a point (assumed on the edge's carrier line)
// against the segment's parameter range.
func LineParamAdmit(e *CEdge, p r3.Vec, tol float64) int {
	dir := e.B.Sub(e.A)
	l := dir.Len()
	u, _ := dir.Normalize()
	return LinWindow{Lo: 0, Hi: l}.Classify(p.Sub(e.A).Dot(u), tol)
}

// CircleAngleAdmit classifies a carrier angle against the arc's window.
func CircleAngleAdmit(e *CEdge, th, tol float64) int {
	return e.Ang.Classify(th, tol/math.Max(e.Radius, 1e-30))
}

// SpineDistOf is the distance from a point to an offset face's spine, with
// the spine foot.
func SpineDistOf(f *CFace, p r3.Vec) (float64, r3.Vec) {
	switch SpineOf(f) {
	case 0:
		return p.Sub(f.Anchor).Len(), f.Anchor
	case 1:
		foot := LinePoint(f.Anchor, f.Axis, p)
		return p.Sub(foot).Len(), foot
	default:
		rel := p.Sub(f.Anchor)
		perp := rel.Sub(f.Axis.Scale(rel.Dot(f.Axis)))
		dir, ok := perp.Normalize()
		if !ok {
			dir = f.RefU
		}
		foot := f.Anchor.Add(dir.Scale(f.Major))
		return p.Sub(foot).Len(), foot
	}
}

// AngleOf is the carrier angle of a direction in the edge's frame.
func AngleOf(e *CEdge, dir r3.Vec) float64 {
	return math.Atan2(dir.Dot(e.RefV), dir.Dot(e.RefU))
}

// RulingContact is one §6 tangential line contact the kernel certified:
// Plane × Cylinder along the tangent ruling, or parallel external
// Cylinder × Cylinder along the common ruling. The plane at offset along
// normal separates the two bodies, and the ruling segment between ends lies
// on both trimmed faces. The kernel keeps the faces and the exact
// feet so the contact layer can publish the manifold of
// docs/contact-geometry-design.md §4.5; Verify reads only the verdict.
type RulingContact struct {
	FaceA, FaceB *CFace
	// normal is the exact unit A-to-B normal of the separating plane. An
	// exactly unit vector with dyadic components is a signed coordinate
	// axis, so this is one.
	Normal proofarith.DyV3
	Offset proofarith.Dyadic
	// ends are the exact ruling ends in lexicographic coordinate order.
	Ends [2]proofarith.DyV3
}

// ExactSignedAxis lifts a carrier direction that is exactly a signed
// coordinate axis.
func ExactSignedAxis(v r3.Vec) (proofarith.DyV3, bool) {
	if _, _, ok := SignedAxis(v); !ok {
		return proofarith.DyV3{}, false
	}
	return proofarith.DyVec(v), true
}

// AxisOfLength returns v/length when v is exactly ±length along one
// coordinate axis and zero on the other two.
func AxisOfLength(v proofarith.DyV3, length proofarith.Dyadic) (proofarith.DyV3, bool) {
	var unit proofarith.DyV3
	found := false
	for i := range 3 {
		switch {
		case v[i].IsZero():
			continue
		case found:
			return proofarith.DyV3{}, false
		case proofarith.DyCmp(v[i], length) == 0:
			unit[i] = proofarith.DyInt(1)
		case proofarith.DyCmp(v[i], proofarith.DyNeg(length)) == 0:
			unit[i] = proofarith.DyInt(-1)
		default:
			return proofarith.DyV3{}, false
		}
		found = true
	}
	return unit, found
}

// DyVecOf lifts a finite carrier vector exactly.
func DyVecOf(v r3.Vec) (proofarith.DyV3, bool) {
	if !proofbound.FiniteVec(v) {
		return proofarith.DyV3{}, false
	}
	return proofarith.DyVec(v), true
}

// DyAxisVec converts an exact vector to its nearest float vector; a signed
// coordinate axis converts exactly.
func DyAxisVec(v proofarith.DyV3) r3.Vec {
	var out r3.Vec
	out.X, _ = v[0].Float64()
	out.Y, _ = v[1].Float64()
	out.Z, _ = v[2].Float64()
	return out
}

func OrderedRulingEnds(ends [2]proofarith.DyV3) [2]proofarith.DyV3 {
	for i := range 3 {
		switch proofarith.DyCmp(ends[0][i], ends[1][i]) {
		case -1:
			return ends
		case 1:
			return [2]proofarith.DyV3{ends[1], ends[0]}
		}
	}
	return ends
}
