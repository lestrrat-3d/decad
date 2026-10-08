package revolvesurvey

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// UndercutRole states whether one revolved face has a normal opposed to the
// pull. The caller resolves each role to the face built from that walk.
type UndercutRole struct {
	Role    string
	Opposes bool
}

// UndercutRoles reads every non-axis meridian wall and each partial-sweep cap
// in the same order as the revolve builder's face roles.
func UndercutRoles(
	loops [][]survey2d.SideWalk, classify AxisClassifier,
	pw, c0, c1, phi0, phi1 float64, full bool, capRoles [2]string,
) []UndercutRole {
	glo, ghi := revolveangle.Extremes(c0, c1, phi0, phi1, full)
	var roles []UndercutRole
	for li, loop := range loops {
		for _, w := range loop {
			if classify.IsAxis(w.SegmentWalk) {
				continue
			}
			mn, mx := math.Inf(1), math.Inf(-1)
			if w.IsCircular() {
				sigma := 1.0
				if w.Th1 < w.Th0 {
					sigma = -1
				}
				lo, hi := math.Min(w.Th0, w.Th1), math.Max(w.Th0, w.Th1)
				for _, g := range []float64{glo, ghi} {
					// n·p = σ(cosθ·pw + sinθ·g) over the meridian.
					a, b := clearance.TrigRange(pw, g, lo, hi)
					mn = math.Min(mn, math.Min(sigma*a, sigma*b))
					mx = math.Max(mx, math.Max(sigma*a, sigma*b))
				}
			} else {
				length := math.Hypot(w.TanInU, w.TanInV)
				nz, nr := w.TanInV/length, -w.TanInU/length
				for _, g := range []float64{glo, ghi} {
					v := nz*pw + nr*g
					mn = math.Min(mn, v)
					mx = math.Max(mx, v)
				}
			}
			roles = append(roles, UndercutRole{
				Role: fmt.Sprintf("side(%d,%d)", li, w.Segs[0]), Opposes: survey2d.OpposesPull(mn, mx),
			})
		}
	}
	if full {
		return roles
	}

	sin0, cos0 := math.Sincos(phi0)
	sin1, cos1 := math.Sincos(phi1)
	// A cap's outward normal opposes the sweep at the start and follows
	// the sweep at the end.
	for _, cap := range []struct {
		role string
		v    float64
	}{
		{role: capRoles[0], v: -(c1*cos0 - c0*sin0)},
		{role: capRoles[1], v: c1*cos1 - c0*sin1},
	} {
		roles = append(roles, UndercutRole{Role: cap.role, Opposes: survey2d.OpposesPull(cap.v, cap.v)})
	}
	return roles
}
