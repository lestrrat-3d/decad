package capband_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/stretchr/testify/require"
)

// sliverCarrier is one wall's offset carrier as the reference reads it: the
// residual of its own equation at X and offset amount t, and its gradient.
type sliverCarrier func(u, v, t float64) (f, gu, gv float64)

// sliverCarrierOf states the wall's offset carrier from its own definition: a
// line moves along rot90 of its direction (n·X = n·start + t), and a circle's
// radius moves to R − t when its walk is counter-clockwise (material inside)
// and to R + t otherwise.
func sliverCarrierOf(w survey2d.SideWalk) sliverCarrier {
	if w.IsCircular() {
		inside := 1.0
		if w.Th1 < w.Th0 {
			inside = -1
		}
		return func(u, v, t float64) (float64, float64, float64) {
			du, dv := u-w.CU, v-w.CV
			r := w.Radius - inside*t
			return du*du + dv*dv - r*r, 2 * du, 2 * dv
		}
	}
	l := math.Hypot(w.EndU-w.StartU, w.EndV-w.StartV)
	nu, nv := -(w.EndV-w.StartV)/l, (w.EndU-w.StartU)/l
	return func(u, v, t float64) (float64, float64, float64) {
		return nu*(u-w.StartU) + nv*(v-w.StartV) - t, nu, nv
	}
}

// referenceCornerShare is the corner share proofbound.ChordLocusVolumeAllow
// owes, computed without the evaluator's closed form or enclosures:
// (ds/dc)·|(v − c) × W| with W = ∫₀^dc (P(t) − Q(t)) dt. The locus P is
// followed by Newton's method on the two carriers' own equations, stepping t
// from the corner, and W is integrated with composite Simpson's rule.
func referenceCornerShare(prev, cur survey2d.SideWalk, cU, cV, vU, vV, dc, ds float64) float64 {
	const n = 4000
	fa, fb := sliverCarrierOf(prev), sliverCarrierOf(cur)
	pu, pv := make([]float64, n+1), make([]float64, n+1)
	u, v := vU, vV
	for i := range n + 1 {
		t := dc * float64(i) / n
		for range 50 {
			f1, a1, b1 := fa(u, v, t)
			f2, a2, b2 := fb(u, v, t)
			det := a1*b2 - a2*b1
			du := (f1*b2 - f2*b1) / det
			dv := (a1*f2 - a2*f1) / det
			u, v = u-du, v-dv
			if math.Abs(du)+math.Abs(dv) < 1e-15*(1+math.Abs(u)+math.Abs(v)) {
				break
			}
		}
		pu[i], pv[i] = u, v
	}
	wu, wv := 0.0, 0.0
	for i := range n + 1 {
		weight := 2.0
		switch {
		case i == 0 || i == n:
			weight = 1
		case i%2 == 1:
			weight = 4
		}
		s := float64(i) / n
		wu += weight * (pu[i] - (pu[0] + s*(pu[n]-pu[0])))
		wv += weight * (pv[i] - (pv[0] + s*(pv[n]-pv[0])))
	}
	h := dc / n
	wu, wv = wu*h/3, wv*h/3
	return ds / dc * math.Abs((vU-cU)*wv-(vV-cV)*wu)
}

func lineWalk(su, sv, eu, ev float64) survey2d.SideWalk {
	l := math.Hypot(eu-su, ev-sv)
	return survey2d.SideWalk{SegmentWalk: survey2d.SegmentWalk{
		StartU: su, StartV: sv, EndU: eu, EndV: ev,
		TanInU: (eu - su) / l, TanInV: (ev - sv) / l, TanOutU: (eu - su) / l, TanOutV: (ev - sv) / l,
	}}
}

func arcWalk(cu, cv, r, th0, th1 float64) survey2d.SideWalk {
	return survey2d.SideWalk{SegmentWalk: survey2d.SegmentWalk{
		Kind: survey2d.WalkCircular,
		CU:   cu, CV: cv, Radius: r, Th0: th0, Th1: th1,
		StartU: cu + r*math.Cos(th0), StartV: cv + r*math.Sin(th0),
		EndU: cu + r*math.Cos(th1), EndV: cv + r*math.Sin(th1),
	}}
}

// TestMiterLocusSliverFluxCoversTheExactCornerShare checks the corner term
// proofbound.ChordLocusVolumeAllow charges for each mitered corner against
// the share a reference computes by following the corner-foot locus
// numerically. The rows are an arc meeting a line at a convex corner turned
// 0.5 rad and 0.01 rad from tangency, the concave arc of a square whose top
// bows into it (material outside the circle), and an asymmetric lens whose
// two arcs meet at a corner, read from each arc's own centre. A line-circle
// corner reads a closed form, so its charge must also sit within 1e-6 of the
// reference; a circle-circle corner reads per-range position and velocity
// enclosures and must cover it and stay within 5% of it (the two lens rows
// sit near 1.02 and 1.01). The 1e-9 slack absorbs the Simpson and Newton
// error.
//
// Shown to fail on 2026-10-09: with MiterLocusSliverFlux answering zero
// every row's charge falls below its reference, with the closed form's
// 6 replaced by 8 every line-circle row does, and with circleCircleSliverMoment
// reading only the hull bound (D·dc²/8 times the arm) the lens rows sit about
// 4 and 8 times their references. At 32 sub-ranges they sit 1.16 and 1.30 times.
func TestMiterLocusSliverFluxCoversTheExactCornerShare(t *testing.T) {
	t.Parallel()
	const alpha = 1.0
	convexCorner := func(gamma float64) (survey2d.SideWalk, survey2d.SideWalk, float64, float64) {
		vU, vV := 10*math.Cos(alpha), 10*math.Sin(alpha)
		du, dv := -math.Sin(alpha+gamma), math.Cos(alpha+gamma)
		return arcWalk(0, 0, 10, -alpha, alpha), lineWalk(vU, vV, vU+20*du, vV+20*dv), vU, vV
	}
	// The lens: circle A (centre (0, −4), radius 10) and circle B (centre
	// (1, 6), radius 12) meet at two corners; the section is inside both.
	lensU, lensV := lensCorner(0, -4, 10, 1, 6, 12)
	thA := math.Atan2(lensV+4, lensU)
	thB := math.Atan2(lensV-6, lensU-1)
	lensA := arcWalk(0, -4, 10, thA-1, thA)
	lensB := arcWalk(1, 6, 12, thB, thB+0.5)
	concaveArc := arcWalk(10, 30.5, 14.5, math.Atan2(-10.5, 10), math.Atan2(-10.5, -10))
	concaveLine := lineWalk(0, 20, 0, 0)

	type row struct {
		name       string
		prev, cur  survey2d.SideWalk
		cU, cV     float64
		vU, vV     float64
		dc, ds     float64
		closedForm bool
	}
	var rows []row
	for _, gamma := range []float64{0.5, 0.01} {
		prev, cur, vU, vV := convexCorner(gamma)
		rows = append(rows,
			row{name: fmt.Sprintf(`convex line-circle turned %v`, gamma), prev: prev, cur: cur,
				vU: vU, vV: vV, dc: 1, ds: 1, closedForm: true},
			row{name: fmt.Sprintf(`convex line-circle turned %v, ds 3dc`, gamma), prev: prev, cur: cur,
				vU: vU, vV: vV, dc: 2, ds: 6, closedForm: true},
		)
	}
	rows = append(rows,
		row{name: `concave arc into a line`, prev: concaveArc, cur: concaveLine, cU: 10, cV: 30.5, vU: 0, vV: 20, dc: 4, ds: 4, closedForm: true},
		row{name: `lens about arc A`, prev: lensA, cur: lensB, cU: 0, cV: -4, vU: lensU, vV: lensV, dc: 1, ds: 1},
		row{name: `lens about arc B`, prev: lensA, cur: lensB, cU: 1, cV: 6, vU: lensU, vV: lensV, dc: 1, ds: 1},
	)
	for _, tc := range rows {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			flux, ok, err := capband.MiterLocusSliverFlux(nil, tc.prev, tc.cur, tc.cU, tc.cV, tc.vU, tc.vV, tc.ds, tc.dc, 0)
			require.NoError(t, err)
			require.True(t, ok, `the corner's sliver must be bounded`)
			want := referenceCornerShare(tc.prev, tc.cur, tc.cU, tc.cV, tc.vU, tc.vV, tc.dc, tc.ds)
			require.Positive(t, want, `the corner's locus is curved`)
			require.GreaterOrEqual(t, flux, want*(1-1e-9),
				`the corner charge %v must cover the reference share %v`, flux, want)
			if tc.closedForm {
				require.LessOrEqual(t, flux, want*(1+1e-6),
					`the closed form's charge %v must match the reference share %v`, flux, want)
				return
			}
			require.LessOrEqual(t, flux, 1.05*want,
				`the per-range charge %v must stay within 5%% of the reference share %v`, flux, want)
		})
	}
}

// lensCorner returns the intersection of two circles with the smaller u.
func lensCorner(au, av, ar, bu, bv, br float64) (float64, float64) {
	du, dv := bu-au, bv-av
	d := math.Hypot(du, dv)
	a := (ar*ar - br*br + d*d) / (2 * d)
	h := math.Sqrt(ar*ar - a*a)
	mu, mv := au+a*du/d, av+a*dv/d
	u0, v0 := mu+h*dv/d, mv-h*du/d
	u1, v1 := mu-h*dv/d, mv+h*du/d
	if u1 < u0 {
		return u1, v1
	}
	return u0, v0
}

// TestMiterLocusSliverFluxZeroOnAStraightLocus checks a corner whose locus is
// straight charges nothing: two lines, and a line tangent to a circle under an
// offset that keeps them tangent (a G1 join's persistent tangency).
func TestMiterLocusSliverFluxZeroOnAStraightLocus(t *testing.T) {
	t.Parallel()
	lineA := lineWalk(0, 0, 10, 0)
	lineB := lineWalk(10, 0, 10, 10)
	flux, ok, err := capband.MiterLocusSliverFlux(nil, lineA, lineB, 0, 0, 10, 0, 1, 1, 0)
	require.NoError(t, err)
	require.True(t, ok)
	require.Zero(t, flux)

	const radius = 1000.0
	line := lineWalk(0, 0, 10, 0)
	circle := arcWalk(0, -radius, radius, math.Pi/2+0.5, math.Pi/2)
	flux, ok, err = capband.MiterLocusSliverFlux(nil, circle, line, 0, -radius, 0, 0, 1, 1, 0)
	require.NoError(t, err)
	require.True(t, ok)
	require.Zero(t, flux)
}
