package exactbool

import (
	"math"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// orientErrBound is Shewchuk's bound (7 + 56ε)ε on the relative error of the float determinant in orient3d, against
// the permanent of the same terms, for ε = 2⁻⁵³.
const orientErrBound = 7.771561172376103e-16

// Orient3D returns the sign of (b−a)×(c−a)·(d−a): +1 when d lies on the side of the plane through a, b and c that the
// normal (b−a)×(c−a) points to, −1 on the other side, 0 when the four points are coplanar. The sign is exact for any
// finite floats.
func Orient3D(a, b, c, d r3.Vec) int {
	adx, ady, adz := a.X-d.X, a.Y-d.Y, a.Z-d.Z
	bdx, bdy, bdz := b.X-d.X, b.Y-d.Y, b.Z-d.Z
	cdx, cdy, cdz := c.X-d.X, c.Y-d.Y, c.Z-d.Z
	bdxcdy, cdxbdy := bdx*cdy, cdx*bdy
	cdxady, adxcdy := cdx*ady, adx*cdy
	adxbdy, bdxady := adx*bdy, bdx*ady
	det := adz*(bdxcdy-cdxbdy) + bdz*(cdxady-adxcdy) + cdz*(adxbdy-bdxady)
	permanent := (math.Abs(bdxcdy)+math.Abs(cdxbdy))*math.Abs(adz) +
		(math.Abs(cdxady)+math.Abs(adxcdy))*math.Abs(bdz) +
		(math.Abs(adxbdy)+math.Abs(bdxady))*math.Abs(cdz)
	bound := orientErrBound * permanent
	if !math.IsInf(det, 0) && !math.IsNaN(det) && (det > bound || -det > bound) {
		// det is Shewchuk's orient3d, which is the negative of the triple product used here.
		if det > 0 {
			return -1
		}
		return 1
	}
	return orient3DExact(a, b, c, d)
}

func orient3DExact(a, b, c, d r3.Vec) int {
	da := proofarith.DyVec(a)
	u := proofarith.DvSub(proofarith.DyVec(b), da)
	v := proofarith.DvSub(proofarith.DyVec(c), da)
	w := proofarith.DvSub(proofarith.DyVec(d), da)
	return proofarith.DvDot(proofarith.DvCross(u, v), w).Sign()
}
