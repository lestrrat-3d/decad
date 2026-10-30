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
	c := CellCornersOf(vLo, vHi, wLo, wHi)
	n := proofarith.Xcross(proofarith.Xsub(c.VHi, c.VLo), proofarith.Xsub(c.WLo, c.VLo))
	if n.X.Sign() == 0 && n.Y.Sign() == 0 && n.Z.Sign() == 0 {
		return 0, false
	}
	boundary := [4]proofarith.Xpt{c.VLo, c.VHi, c.WHi, c.WLo}
	for i := range boundary {
		in := proofarith.Xsub(boundary[i], boundary[(i+3)%4])
		out := proofarith.Xsub(boundary[(i+1)%4], boundary[i])
		if proofarith.XdotSign(n, proofarith.Xcross(in, out)) <= 0 {
			return 0, false
		}
	}
	m := proofarith.Xsub(c.WHi, c.VLo)
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
