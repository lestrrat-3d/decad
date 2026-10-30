package proofbound

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// CellHeightOffsetUpper is a second bound on the quantity
// CellTwistOffsetUpper bounds: how far the held triangle pair of a cell
// strays from the bilinear patch over the same four corners, as a
// Hausdorff distance between the two surfaces. It answers false when its
// precondition fails, and the caller then keeps CellTwistOffsetUpper.
//
// The corners are the cell's rings k and k+1 at its two columns: vLo, vHi on
// ring k, wLo, wHi on ring k+1, so the cell's boundary runs vLo → vHi → wHi →
// wLo. Let n = (vHi − vLo) × (wLo − vLo), the normal of the plane Π through
// vLo, vHi and wLo, and h(p) = n̂·(p − vLo) the height over Π. The
// precondition is that the cell projected onto Π is a strictly convex
// quadrilateral: n·(in × out) > 0 at each of the four corners, where in and
// out are the boundary edges arriving at and leaving the corner. Projection
// along n̂ leaves n·(x × y) unchanged, so these are the projected turns.
//
// Under it:
//
//   - The bilinear patch Q(s, u) has ∂Q/∂s × ∂Q/∂u affine in (s, u), and its
//     n component at the four corners of the unit square is the four corner
//     turns above, so it is positive on the whole square. The projected
//     patch is then a local homeomorphism whose boundary runs once around
//     the convex quadrilateral K, so it is a homeomorphism onto K.
//   - Either diagonal splits a strictly convex K into two triangles that tile
//     it, so the projected triangle pair is K as well.
//   - Both surfaces are therefore graphs over K, with the same boundary.
//     h is 0 at vLo, vHi and wLo and H = h(wHi) at the fourth corner.
//     h∘Q(s, u) = s·u·H, and a triangle's heights interpolate its corners',
//     so both graphs lie between heights 0 and H.
//   - Two points of the two graphs above one point of K are at most |H|
//     apart, and the map pairing them is the identity on the boundary.
//
// So |H| = |n·(wHi − vLo)| / |n| bounds the cell's offset. It is the distance
// of wHi from Π, and zero for a planar cell however far its corners move
// within its plane. Every quantity is exact rational arithmetic over the held
// floats, and the square root rounds up through RatSqrtUp, so the bound
// covers the vertices exactly as the mesh stores them. A non-finite corner
// answers +Inf and false.
func CellHeightOffsetUpper(vLo, vHi, wLo, wHi r3.Vec) (float64, bool) {
	if !FiniteVec(vLo) || !FiniteVec(vHi) || !FiniteVec(wLo) || !FiniteVec(wHi) {
		return math.Inf(1), false
	}
	corners := CellCornersOf(vLo, vHi, wLo, wHi)
	a, b, c, d := corners.VLo, corners.VHi, corners.WHi, corners.WLo
	e := proofarith.Xsub(b, a)
	f := proofarith.Xsub(c, d)
	p := proofarith.Xsub(d, a)
	q := proofarith.Xsub(c, b)
	n := proofarith.Xcross(e, p)
	if n.X.Sign() == 0 && n.Y.Sign() == 0 && n.Z.Sign() == 0 {
		return 0, false
	}
	// The turn at A is n·n, which is positive after the nonzero check.
	// These are the other three projected turns, in boundary order B, C, D.
	if proofarith.XdotSign(n, proofarith.Xcross(e, q)) <= 0 ||
		proofarith.XdotSign(n, proofarith.Xcross(f, q)) <= 0 ||
		proofarith.XdotSign(n, proofarith.Xcross(f, p)) <= 0 {
		return 0, false
	}
	m := proofarith.Xsub(c, a)
	// n·m = dot / (n.W·m.W) and n·n = nn / n.W², so
	// H² = (n·m)² / (n·n) = dot² / (m.W²·nn).
	dot := proofarith.XdotNum(n, m)
	if dot.Sign() == 0 {
		return 0, true
	}
	num := new(big.Int).Mul(dot, dot)
	den := new(big.Int).Mul(m.W, m.W)
	den.Mul(den, proofarith.XdotNum(n, n))
	return RatSqrtUp(new(big.Rat).SetFrac(num, den)), true
}
