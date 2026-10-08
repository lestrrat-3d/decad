package facetproof

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// LowerSupport records one exact source-box lower face retained by a mesh
// union. The other operand's certified lower bound is strictly higher.
type LowerSupport struct {
	Plane          proof.Dyadic
	FootLo, FootHi [2]proof.Dyadic
	SourceGroup    int
}

// AxisSupportInput contains the faceted payload and the source-face admission
// flags needed to prove one complete rectangular axis support face.
type AxisSupportInput struct {
	Verts, ExactSourceVerts []r3.Vec
	Tris, ExactSourceTris   [][3]int
	FaceOf, SourceGroup     []int
	FaceValid               []bool
	GroupCount              int
	MeshBound, VolSymDiff   float64
	Xform, Pose             r3.Transform
	Lower                   *LowerSupport
	Axis, Side              int
}

// AxisSupportProof identifies the one admitted source face by its index.
// The caller maps it back to topology after this numeric proof succeeds.
type AxisSupportProof struct {
	FaceIndex        int
	Axis, Side       int
	Plane            proof.Dyadic
	FootLo, FootHi   [2]proof.Dyadic
	OuterLo, OuterHi [3]proof.Dyadic
	Corners          [4]proof.DyV3
	Normal           r3.Vec
}

// TranslationOnly reports a valid finite rigid translation with identity
// basis. It also gates preservation of an exact source mesh by placement.
func TranslationOnly(t r3.Transform) bool {
	return t.IsValid() && proofbound.FiniteVec(t.Translation()) && t.Basis() == r3.Identity().Basis()
}

// ProveAxisSupport verifies that one outward-facing planar source face is
// the complete rectangular contact set at an axis extremum. A zero-bound
// mesh, a saved exact source mesh after translation, or a retained exact
// lower source-box face may supply that support.
func ProveAxisSupport(ctx context.Context, in AxisSupportInput) (AxisSupportProof, bool, error) {
	if in.Axis < 0 || in.Axis > 2 || (in.Side != 0 && in.Side != 1) ||
		!finite(in.MeshBound, in.VolSymDiff) || in.MeshBound < 0 || in.VolSymDiff < 0 ||
		len(in.Verts) == 0 || len(in.Tris) == 0 || len(in.FaceOf) != len(in.Tris) {
		return AxisSupportProof{}, false, nil
	}
	sourceVerts := in.Verts
	placedFromSource := false
	certifiedUnion := in.Lower != nil && in.Xform == r3.Identity() &&
		in.Axis == 2 && in.Side == 0 && TranslationOnly(in.Pose)
	if certifiedUnion && (len(in.SourceGroup) != len(in.Tris) ||
		in.Lower.SourceGroup < 0 || in.Lower.SourceGroup >= in.GroupCount) {
		return AxisSupportProof{}, false, nil
	}
	if in.MeshBound != 0 || in.VolSymDiff != 0 {
		if !certifiedUnion {
			if !TranslationOnly(in.Xform) ||
				len(in.ExactSourceVerts) != len(in.Verts) ||
				len(in.ExactSourceTris) != len(in.Tris) {
				return AxisSupportProof{}, false, nil
			}
			for i, tri := range in.Tris {
				if tri != in.ExactSourceTris[i] {
					return AxisSupportProof{}, false, nil
				}
			}
			sourceVerts, placedFromSource = in.ExactSourceVerts, true
		}
	}
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return AxisSupportProof{}, false, err
	}
	placed := make([]proof.DyV3, len(in.Verts))
	out := AxisSupportProof{FaceIndex: -1, Axis: in.Axis, Side: in.Side}
	for i, v := range sourceVerts {
		if err := budget.Step(); err != nil {
			return AxisSupportProof{}, false, err
		}
		if !proofbound.FiniteVec(v) {
			return AxisSupportProof{}, false, nil
		}
		source := proof.DyVec(v)
		if placedFromSource {
			if !proofbound.FiniteVec(in.Verts[i]) {
				return AxisSupportProof{}, false, nil
			}
			source = transformDy(in.Xform, source)
			difference := proof.DvSub(source, proof.DyVec(in.Verts[i]))
			bound := proof.MustDyOf(in.MeshBound)
			if proof.DyCmp(proof.DvDot(difference, difference), proof.DyMul(bound, bound)) > 0 {
				return AxisSupportProof{}, false, nil
			}
		}
		placed[i] = transformDy(in.Pose, source)
		for j := range 3 {
			if i == 0 || proof.DyCmp(placed[i][j], out.OuterLo[j]) < 0 {
				out.OuterLo[j] = placed[i][j]
			}
			if i == 0 || proof.DyCmp(placed[i][j], out.OuterHi[j]) > 0 {
				out.OuterHi[j] = placed[i][j]
			}
		}
	}
	out.Plane = out.OuterLo[in.Axis]
	sign := -1
	if in.Side == 1 {
		out.Plane, sign = out.OuterHi[in.Axis], 1
	}
	if certifiedUnion {
		out.Plane = proof.DyAdd(in.Lower.Plane, proof.MustDyOf(in.Pose.Translation().Z))
		bound := proof.MustDyOf(in.MeshBound)
		if proof.DyCmp(proof.DyAdd(out.OuterLo[2], bound), out.Plane) < 0 {
			return AxisSupportProof{}, false, nil
		}
		for j := range 3 {
			out.OuterLo[j] = proof.DySubScalar(out.OuterLo[j], bound)
			out.OuterHi[j] = proof.DyAdd(out.OuterHi[j], bound)
		}
	}
	windingSign := sign
	if in.Pose.IsReflection() {
		windingSign = -windingSign
	}
	var projected [2]int
	for j, n := 0, 0; j < 3; j++ {
		if j != in.Axis {
			projected[n] = j
			n++
		}
	}
	covered := make([]bool, len(placed))
	var area2 proof.Dyadic
	first := true
	for i, tri := range in.Tris {
		if err := budget.Step(); err != nil {
			return AxisSupportProof{}, false, err
		}
		for _, vertex := range tri {
			if vertex < 0 || vertex >= len(placed) {
				return AxisSupportProof{}, false, nil
			}
		}
		faceIndex := in.FaceOf[i]
		if faceIndex < 0 || faceIndex >= len(in.FaceValid) {
			return AxisSupportProof{}, false, nil
		}
		if proof.DyCmp(placed[tri[0]][in.Axis], out.Plane) != 0 ||
			proof.DyCmp(placed[tri[1]][in.Axis], out.Plane) != 0 ||
			proof.DyCmp(placed[tri[2]][in.Axis], out.Plane) != 0 {
			continue
		}
		if certifiedUnion && in.SourceGroup[i] != in.Lower.SourceGroup {
			return AxisSupportProof{}, false, nil
		}
		if !in.FaceValid[faceIndex] || (!first && out.FaceIndex != faceIndex) {
			return AxisSupportProof{}, false, nil
		}
		out.FaceIndex = faceIndex
		cross := proof.DvCross(proof.DvSub(placed[tri[1]], placed[tri[0]]),
			proof.DvSub(placed[tri[2]], placed[tri[0]]))
		if cross[in.Axis].Sign() != windingSign ||
			!cross[projected[0]].IsZero() || !cross[projected[1]].IsZero() {
			return AxisSupportProof{}, false, nil
		}
		if windingSign < 0 {
			area2 = proof.DySubScalar(area2, cross[in.Axis])
		} else {
			area2 = proof.DyAdd(area2, cross[in.Axis])
		}
		for _, vertex := range tri {
			covered[vertex] = true
			for j, coord := range projected {
				if first || proof.DyCmp(placed[vertex][coord], out.FootLo[j]) < 0 {
					out.FootLo[j] = placed[vertex][coord]
				}
				if first || proof.DyCmp(placed[vertex][coord], out.FootHi[j]) > 0 {
					out.FootHi[j] = placed[vertex][coord]
				}
			}
			first = false
		}
	}
	if out.FaceIndex < 0 || proof.DyCmp(out.FootLo[0], out.FootHi[0]) >= 0 ||
		proof.DyCmp(out.FootLo[1], out.FootHi[1]) >= 0 {
		return AxisSupportProof{}, false, nil
	}
	if certifiedUnion {
		for j := range 2 {
			if proof.DyCmp(out.FootLo[j], in.Lower.FootLo[j]) != 0 ||
				proof.DyCmp(out.FootHi[j], in.Lower.FootHi[j]) != 0 {
				return AxisSupportProof{}, false, nil
			}
		}
	}
	// A source face must name this support patch in full.
	for i, tri := range in.Tris {
		if err := budget.Step(); err != nil {
			return AxisSupportProof{}, false, err
		}
		if in.FaceOf[i] != out.FaceIndex {
			continue
		}
		for _, vertex := range tri {
			if proof.DyCmp(placed[vertex][in.Axis], out.Plane) != 0 {
				return AxisSupportProof{}, false, nil
			}
		}
	}
	for i := range placed {
		if err := budget.Step(); err != nil {
			return AxisSupportProof{}, false, err
		}
		if proof.DyCmp(placed[i][in.Axis], out.Plane) == 0 && !covered[i] {
			return AxisSupportProof{}, false, nil
		}
	}
	width := proof.DySubScalar(out.FootHi[0], out.FootLo[0])
	height := proof.DySubScalar(out.FootHi[1], out.FootLo[1])
	if proof.DyCmp(area2, proof.DyMul(proof.MustDyOf(2), proof.DyMul(width, height))) != 0 {
		return AxisSupportProof{}, false, nil
	}
	for i, corner := range [][2]int{{0, 0}, {1, 0}, {1, 1}, {0, 1}} {
		out.Corners[i][in.Axis] = out.Plane
		for j, coord := range projected {
			out.Corners[i][coord] = out.FootLo[j]
			if corner[j] == 1 {
				out.Corners[i][coord] = out.FootHi[j]
			}
		}
	}
	switch in.Axis {
	case 0:
		out.Normal.X = float64(sign)
	case 1:
		out.Normal.Y = float64(sign)
	case 2:
		out.Normal.Z = float64(sign)
	}
	return out, true, budget.Err()
}

func finite(values ...float64) bool {
	for _, value := range values {
		if proofbound.IsNonFinite(value) {
			return false
		}
	}
	return true
}

func transformDy(t r3.Transform, p proof.DyV3) proof.DyV3 {
	b := t.Basis()
	scale := func(v proof.DyV3, s proof.Dyadic) proof.DyV3 {
		return proof.DyV3{proof.DyMul(v[0], s), proof.DyMul(v[1], s), proof.DyMul(v[2], s)}
	}
	return proof.DvAdd(proof.DyVec(t.Translation()), proof.DvAdd(scale(proof.DyVec(b.EX), p[0]),
		proof.DvAdd(scale(proof.DyVec(b.EY), p[1]), scale(proof.DyVec(b.EZ), p[2]))))
}
