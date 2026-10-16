package main

import (
	"context"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
)

type threadDiagnosticKey struct{}
type threadDiagnosticState struct {
	t        *testing.T
	recorded bool
}

// TestThreadVolumeDiagnostic executes the exact producer with a reporting hook
// before its unchanged verification gate. A passing diagnostic test is not a
// passing producer, a Sound report, or approval to publish an image.
func TestThreadVolumeDiagnostic(t *testing.T) {
	state := &threadDiagnosticState{t: t}
	ctx := context.WithValue(t.Context(), threadDiagnosticKey{}, state)
	_, err := externalThread(ctx)
	if err != nil {
		t.Logf("unchanged_producer_rejected=%q", err.Error())
	} else {
		t.Log("unchanged_producer_rejected=false")
	}
	if !state.recorded {
		t.Fatal("diagnostic hook was not reached")
	}
	t.Log("diagnostic_only=true publication_approved=false")
}

func recordThreadDiagnostic(ctx context.Context, body *decad.Body, report *decad.Report) {
	state := ctx.Value(threadDiagnosticKey{}).(*threadDiagnosticState)
	t := state.t
	state.recorded = true
	t.Logf("report_status=%s relative_tolerance=%.17g bodies=%d diagnostics=%d",
		report.Status, report.Request.RelativeTolerance.Base(), len(report.Bodies), len(report.Diagnostics))
	if len(report.Bodies) != 1 || report.Bodies[0].Region == nil {
		t.Error("expected one solid body report")
		return
	}
	r := report.Bodies[0]
	t.Logf("validity=%s body_status=%s lumps=%d voids=%d shells=%d",
		r.Validity.Outcome, r.Status, r.Topology.Lumps, r.Topology.Voids, len(body.Shells()))
	if r.Validity.Outcome != decad.ValidityValid {
		t.Error("held-boundary validity is not valid")
	}
	for _, reading := range []struct {
		name  string
		value decad.ScalarReading
	}{
		{"volume", r.Region.Volume}, {"area", r.Area},
	} {
		v := reading.value
		t.Logf("reading=%s value=%.17g bound=%.17g exactness=%s tolerance=%s",
			reading.name, v.Value.Base(), v.Bound.Base(), v.Exactness, v.Tolerance.State)
		if v.Tolerance.Limit != nil {
			t.Logf("reading=%s tolerance_limit=%.17g", reading.name, v.Tolerance.Limit.Base())
		}
	}
	c := r.Region.Centroid
	t.Logf("centroid=(%.17g,%.17g,%.17g) bound=%.17g exactness=%s tolerance=%s",
		c.Value.X, c.Value.Y, c.Value.Z, c.Bound.Base(), c.Exactness, c.Tolerance.State)
	if c.Tolerance.Limit != nil {
		t.Logf("centroid_tolerance_limit=%.17g", c.Tolerance.Limit.Base())
	}
	if len(report.Diagnostics) > 16 {
		t.Error("diagnostic count exceeds reporting cap of 16")
		return
	}
	for i, d := range report.Diagnostics {
		t.Logf("diagnostic=%d code=%s reading=%s status=%s", i, d.Code, d.Reading, d.Status)
		if d.Required != nil {
			t.Logf("diagnostic=%d required=%.17g", i, d.Required.Base())
		}
		if d.Observed != nil {
			t.Logf("diagnostic=%d observed=%.17g bound=%.17g", i, d.Observed.Value.Base(), d.Observed.Bound.Base())
		}
		if d.ObservedVec != nil {
			t.Logf("diagnostic=%d observed_vec=(%.17g,%.17g,%.17g) bound=%.17g", i, d.ObservedVec.Value.X, d.ObservedVec.Value.Y, d.ObservedVec.Value.Z, d.ObservedVec.Bound.Base())
		}
	}
	coefficient := clippedThreadVolumeOverPi()
	piLo, piHi := diagnosticPiBounds()
	expectedLo := new(big.Rat).Mul(coefficient, piLo)
	expectedHi := new(big.Rat).Mul(coefficient, piHi)
	v := r.Region.Volume
	value, bound := diagnosticRat(v.Value.Base()), diagnosticRat(v.Bound.Base())
	publishedLo := new(big.Rat).Sub(value, bound)
	publishedHi := new(big.Rat).Add(value, bound)
	contained := publishedLo.Cmp(expectedLo) <= 0 && publishedHi.Cmp(expectedHi) >= 0
	t.Logf("independent_volume_over_pi=%s", coefficient.RatString())
	t.Logf("independent_volume_interval_mm3_rounded_30dp=[%s,%s]", expectedLo.FloatString(30), expectedHi.FloatString(30))
	t.Logf("published_volume_interval_mm3_rounded_30dp=[%s,%s] contains_independent_interval=%t",
		publishedLo.FloatString(30), publishedHi.FloatString(30), contained)
	delta := new(big.Rat).Sub(value, new(big.Rat).Quo(new(big.Rat).Add(expectedLo, expectedHi), big.NewRat(2, 1)))
	t.Logf("volume_value_minus_independent_midpoint_mm3=%s", delta.FloatString(30))
	if !contained {
		t.Error("published volume bound does not enclose the independent clipped-volume interval")
	}
	// Run the existing visual probes separately, even though the original
	// producer will still reject its unexpected diagnostic after this hook.
	if err := checkThreadGroove(ctx, body); err != nil {
		t.Errorf("held_mesh_probes_failed=%s", err)
	} else {
		t.Log("held_mesh_probes_passed=true certified_true_solid_membership=false")
	}
}

func diagnosticRat(x float64) *big.Rat { return new(big.Rat).SetFloat64(x) }

// Integrate the recorded triangle inside rho<=5. Its screw-sweep Jacobian
// is rho, k=pitch/(2*pi), and theta ranges from max(0,-zeta/k) to 16*pi.
// Q=int rho dA and Mminus=int_{zeta<0} rho*zeta dA give removed volume
// pi*(16*Q + 4/3*Mminus). The R5,L14 blank contributes 350*pi.
// The clipped profile width is below pitch, so turns do not overlap; its
// highest point after eight turns is below 14, so only y=0 clips the sweep.
// This uses exact rationals of the actual rounded sketch coordinates, not
// the boolean mesh, measured result, numerical quadrature, or kernel bounds.
func clippedThreadVolumeOverPi() *big.Rat {
	const start = -0.375
	const halfWidth = 0.6928203230275509
	groove := [][2]*big.Rat{
		{diagnosticRat(4.1), diagnosticRat(start)},
		{diagnosticRat(5.3), diagnosticRat(start - halfWidth)},
		{diagnosticRat(5.3), diagnosticRat(start + halfWidth)},
	}
	inside := diagnosticClip(groove, func(p [2]*big.Rat) *big.Rat { return new(big.Rat).Sub(diagnosticRat(5), p[0]) })
	q, _ := diagnosticMoments(inside)
	_, below := diagnosticMoments(diagnosticClip(inside, func(p [2]*big.Rat) *big.Rat { return new(big.Rat).Neg(p[1]) }))
	removed := new(big.Rat).Mul(big.NewRat(16, 1), q)
	removed.Add(removed, new(big.Rat).Mul(big.NewRat(4, 3), below))
	return new(big.Rat).Sub(big.NewRat(350, 1), removed)
}

// Each alternating arctangent partial sum and its next term bound atan(1/d).
// Machin's identity pi=16*atan(1/5)-4*atan(1/239) then supplies rational bounds.
func diagnosticPiBounds() (*big.Rat, *big.Rat) {
	atan := func(d int64, n int) (*big.Rat, *big.Rat) {
		x := big.NewRat(1, d)
		x2 := new(big.Rat).Mul(x, x)
		power, sum := new(big.Rat).Set(x), new(big.Rat)
		for i := 0; i < n; i++ {
			term := new(big.Rat).Quo(power, big.NewRat(int64(2*i+1), 1))
			if i%2 == 0 {
				sum.Add(sum, term)
			} else {
				sum.Sub(sum, term)
			}
			power.Mul(power, x2)
		}
		next := new(big.Rat).Quo(power, big.NewRat(int64(2*n+1), 1))
		other := new(big.Rat).Set(sum)
		if n%2 == 0 {
			other.Add(other, next)
			return sum, other
		}
		other.Sub(other, next)
		return other, sum
	}
	aLo, aHi := atan(5, 80)
	bLo, bHi := atan(239, 24)
	lo := new(big.Rat).Sub(new(big.Rat).Mul(big.NewRat(16, 1), aLo), new(big.Rat).Mul(big.NewRat(4, 1), bHi))
	hi := new(big.Rat).Sub(new(big.Rat).Mul(big.NewRat(16, 1), aHi), new(big.Rat).Mul(big.NewRat(4, 1), bLo))
	return lo, hi
}

func diagnosticMoments(pts [][2]*big.Rat) (*big.Rat, *big.Rat) {
	q, m := new(big.Rat), new(big.Rat)
	mul := func(a, b *big.Rat) *big.Rat { return new(big.Rat).Mul(a, b) }
	for i := range pts {
		x0, y0 := pts[i][0], pts[i][1]
		x1, y1 := pts[(i+1)%len(pts)][0], pts[(i+1)%len(pts)][1]
		c := new(big.Rat).Sub(mul(x0, y1), mul(x1, y0))
		q.Add(q, mul(new(big.Rat).Add(x0, x1), c))
		mixed := new(big.Rat).Add(mul(x0, y1), mul(x1, y0))
		mixed.Add(mixed, mul(big.NewRat(2, 1), new(big.Rat).Add(mul(x0, y0), mul(x1, y1))))
		m.Add(m, mul(mixed, c))
	}
	q.Quo(q, big.NewRat(6, 1))
	m.Quo(m, big.NewRat(24, 1))
	if q.Sign() < 0 {
		q.Neg(q)
		m.Neg(m)
	}
	return q, m
}

// diagnosticClip keeps the part of a polygon where keep(p) ≥ 0 for an affine
// keep, Sutherland–Hodgman over exact rationals.
func diagnosticClip(pts [][2]*big.Rat, keep func(p [2]*big.Rat) *big.Rat) [][2]*big.Rat {
	var out [][2]*big.Rat
	for i := range pts {
		a, b := pts[i], pts[(i+1)%len(pts)]
		fa, fb := keep(a), keep(b)
		if fa.Sign() >= 0 {
			out = append(out, a)
		}
		if fa.Sign()*fb.Sign() < 0 {
			t := new(big.Rat).Quo(fa, new(big.Rat).Sub(fa, fb))
			var p [2]*big.Rat
			for k := range 2 {
				p[k] = new(big.Rat).Add(a[k], new(big.Rat).Mul(t, new(big.Rat).Sub(b[k], a[k])))
			}
			out = append(out, p)
		}
	}
	return out
}
