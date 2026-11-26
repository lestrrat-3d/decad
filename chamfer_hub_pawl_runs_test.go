package decad_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad"
)

const (
	hubBottom  = 4.25
	hubHeight  = 10.0
	hubChamfer = 0.5
)

// hubBody extrudes the one-way clutch hub's outline between hubBottom and hubBottom + hubHeight: the outer loop of
// hub, the D bore, and the three bolt holes.
func hubBody(t *testing.T, hub []hubSeg) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), hubBottom)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	pts := make([]*sketch.Point, len(hub))
	for i, h := range hub {
		pts[i] = s.CreatePoint(h.x0, h.y0)
		s.Fix(pts[i])
	}
	for i, h := range hub {
		next := pts[(i+1)%len(hub)]
		if !h.arc {
			s.CreateLine(pts[i], next)
			continue
		}
		c := s.CreatePoint(h.cx, h.cy)
		s.Fix(c)
		if h.ccw {
			s.CreateArc(c, pts[i], next)
		} else {
			s.CreateArc(c, next, pts[i])
		}
	}
	// The D bore, with its flat on top, and three bolt holes.
	const flatX, flatY, boreR = 1.51657508881031, 2.0499999999999998, 2.5499999999999998
	origin := s.CreatePoint(0, 0)
	a, b := s.CreatePoint(-flatX, flatY), s.CreatePoint(flatX, flatY)
	s.Fix(origin)
	s.Fix(a)
	s.Fix(b)
	s.CreateLine(a, b)
	s.CreateArc(origin, a, b)
	for _, c := range [][2]float64{{7.274613391789285, 4.2}, {-7.274613391789285, 4.2}, {0, -8.4}} {
		p := s.CreatePoint(c[0], c[1])
		s.Fix(p)
		s.CreateCircle(p, 1.3)
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var pr *sketch.Profile
	for _, p := range s.Profiles() {
		if p.Valid && len(p.Holes) == 4 {
			pr = p
		}
	}
	require.NotNil(t, pr)
	body, err := decad.Extrude(t.Context(), decad.New(), s, pr, decad.Distance{D: units.Millimeters(hubHeight), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// hubSelection selects, on each of the hub's caps, the edges of the outer loop that hub marks selected, by the end points
// the body stores for them.
func hubSelection(t *testing.T, body *decad.Body, hub []hubSeg) (*decad.EdgeQuery, int) {
	t.Helper()
	near := func(p r3.Vec, x, y float64) bool { return math.Hypot(p.X-x, p.Y-y) < 1e-9 }
	var q *decad.EdgeQuery
	count := 0
	for _, e := range body.Edges() {
		s, en := e.Start().Position().Value, e.End().Position().Value
		if s.Z != en.Z || (s.Z != hubBottom && s.Z != hubBottom+hubHeight) {
			continue
		}
		for _, h := range hub {
			if !h.sel || !(near(s, h.x0, h.y0) && near(en, h.x1, h.y1) || near(s, h.x1, h.y1) && near(en, h.x0, h.y0)) {
				continue
			}
			pred := decad.Circular()
			if !h.arc {
				pred = decad.ParallelTo(en.Sub(s))
			}
			if q == nil {
				q = decad.Edges(decad.EndpointAt(s), pred)
			} else {
				q = q.Or(decad.EndpointAt(s), pred)
			}
			count++
		}
	}
	require.NotNil(t, q)
	return q, count
}

// TestChamferHubPawlRunsClockwise chamfers the three pawl chains of both caps of the one-way clutch hub in one call: each chain is
// an arm's outer arc, the web's radial line and its round, the slot-bottom arc, the slot's round and the arm's inner arc,
// whose outer arc meets the radial line in a sharp 90 degree corner beside an arc. Sections of the result lie on the
// bevel there.
func TestChamferHubPawlRunsClockwise(t *testing.T) { chamferHubPawlRuns(t, hubLoop605zzCw) }

// TestChamferHubPawlRunsCounterClockwise is TestChamferHubPawlRunsClockwise for the hub whose arms turn the other way.
func TestChamferHubPawlRunsCounterClockwise(t *testing.T) { chamferHubPawlRuns(t, hubLoop605zzCcw) }

func chamferHubPawlRuns(t *testing.T, hub []hubSeg) {
	t.Parallel()
	{
		{
			body := hubBody(t, hub)
			sel, count := hubSelection(t, body, hub)
			require.Equal(t, 36, count)
			out, err := decad.Chamfer(body, sel, units.Millimeters(hubChamfer)).Do(t.Context())
			require.NoError(t, err)
			require.True(t, out.IsSolid())
			mesh, err := out.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			require.True(t, mesh.BoundaryVerified())

			// The loop walks counter-clockwise, so its material lies to the left of each segment.
			area := 0.0
			for _, h := range hub {
				area += h.x0*h.y1 - h.x1*h.y0
			}
			side := 1.0
			if area < 0 {
				side = -1
			}
			for i, h := range hub {
				if h.arc || !h.sel || !hub[(i+len(hub)-1)%len(hub)].arc || hub[(i+len(hub)-1)%len(hub)].r != hub[(i+len(hub)-1)%len(hub)].r {
					continue
				}
				prev := hub[(i+len(hub)-1)%len(hub)]
				if math.Abs(prev.r-13.95) > 1e-6 {
					continue
				}
				// The corner W of the outer arc and the radial line: the bevel's offset line meets the arc's offset
				// circle at the setback t = d - z below the cap.
				ux, uy := h.x1-h.x0, h.y1-h.y0
				ln := math.Hypot(ux, uy)
				ux, uy = ux/ln, uy/ln
				nx, ny := -uy*side, ux*side
				for _, z := range []float64{0.1, 0.25, 0.4} {
					tt := hubChamfer - z
					frame, err := r3.NewFrame(r3.Vec{Z: hubBottom + hubHeight - z}, r3.Vec{X: 1}, r3.Vec{Y: 1})
					require.NoError(t, err)
					o, err := out.SectionOutline(t.Context(), frame, units.Millimeters(0.03))
					require.NoError(t, err)
					off, err := o.OffSurface.In(units.Millimeter)
					require.NoError(t, err)
					// The offset line is p = W + t n + s u; solve |p|^2 = (r - t)^2 for the root nearest W.
					px, py := h.x0+nx*tt, h.y0+ny*tt
					bq := px*ux + py*uy
					disc := bq*bq - (px*px + py*py - (13.95-tt)*(13.95-tt))
					require.Positive(t, disc)
					var best [2]float64
					bestS := math.Inf(1)
					for _, s := range []float64{-bq - math.Sqrt(disc), -bq + math.Sqrt(disc)} {
						if math.Abs(s) < bestS {
							bestS, best = math.Abs(s), [2]float64{px + s*ux, py + s*uy}
						}
					}
					require.Less(t, nearestLoopPoint(o, best[0], best[1]), off+1e-3, fmt.Sprintf("W at depth %v, segment %d", z, i))
					// The points of the arc's bevel, away from its ends, lie on the offset circle.
					a0 := math.Atan2(prev.y0-prev.cy, prev.x0-prev.cx)
					a1 := math.Atan2(prev.y1-prev.cy, prev.x1-prev.cx)
					for a1 < a0 {
						a1 += 2 * math.Pi
					}
					rho := 13.95 - tt
					on := 0
					for _, loop := range o.Loops {
						for _, p := range loop {
							phi := math.Atan2(p.Y, p.X)
							for phi < a0 {
								phi += 2 * math.Pi
							}
							if phi > a1-0.1 || phi < a0+0.1 || math.Abs(math.Hypot(p.X, p.Y)-rho) > 0.2 {
								continue
							}
							on++
							require.InDelta(t, rho, math.Hypot(p.X, p.Y), off+1e-3, "depth %v, segment %d", z, i)
						}
					}
					require.Greater(t, on, 10)
				}
			}
		}
	}
}
