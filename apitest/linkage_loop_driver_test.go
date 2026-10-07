package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// rockerFromCoupler is the crank-rocker's configuration when its coupler's
// joint, the coupler's turn relative to the crank, stands at qc radians from
// the zero pose: the crank angle θ2 and the follower angle θ4. With the
// crank-coupler angle φ = φ0 + qc fixed, the pin B = e^{iθ2}·(r + l·e^{iφ})
// lies on the circle of radius |r + l·e^{iφ}| about O2 and on the follower's
// circle of radius f about O4; on the branch above the ground line B's angle
// gives θ2 = arg B − arg(r + l·e^{iφ}).
func rockerFromCoupler(qc float64) (float64, float64) {
	g, r, l, f := rockerGround, rockerCrank, rockerCoupler, rockerFollower
	t4 := rockerTheta4(0)
	bx0, by0 := g+f*math.Cos(t4), f*math.Sin(t4)
	phi := math.Atan2(by0, bx0-r) + qc
	wx, wy := r+l*math.Cos(phi), l*math.Sin(phi)
	d := math.Hypot(wx, wy)
	// The circles |B| = d and |B − O4| = f meet above the ground line at
	// x = (d² − f² + g²)/(2g).
	x := (d*d - f*f + g*g) / (2 * g)
	y := math.Sqrt(d*d - x*x)
	th2 := math.Atan2(y, x) - math.Atan2(wy, wx)
	th4 := math.Atan2(y, x-g)
	return th2, th4
}

// TestVerifyLinkageLoopDrivenAtTheCoupler pins docs/linkage-check-design.md
// §15.1-§15.2's driver below Common on scene 7's crank-rocker, driven at the
// coupler's joint, whose parent is the crank: the driving angle runs from the
// crank's line, offset by its zero-pose reading r₀, which a probe scene
// reads. Driven 0° → 30° and 0° → −30°, every pose's crank value is the closed
// form θ2(qc) and the follower's θ4(qc) − θ4(0), within 1e-9 rad, each with a
// positive bound below 1e-9; the coupler carries its stated value; a schedule
// poses what the report evaluated; and the drive reads Sound.
//
// Leg seen to fail when deleted: the probe's offset (the driver is asked at
// the angle 0 from the crank's line, a configuration the document's pins do
// not hold, and the zero pose's falsifier refuses the loop).
func TestVerifyLinkageLoopDrivenAtTheCoupler(t *testing.T) {
	t.Parallel()
	for _, to := range []float64{30, -30} {
		t.Run(units.Degrees(to).String(), func(t *testing.T) {
			t.Parallel()
			fb, _ := buildRocker(t, false)
			drive := decad.Drive{{Link: fb.couplerLk, From: units.Degrees(0), To: units.Degrees(to)}}
			report := verifyLinkage(t, fb.doc, fb.linkage, drive)
			require.Equal(t, decad.Sound, report.Status)
			require.NotEmpty(t, report.Poses)
			sched, err := fb.linkage.Schedule(t.Context(), drive)
			require.NoError(t, err)
			_, t40 := rockerFromCoupler(0)
			for _, p := range report.Poses {
				s := p.Pose.At.Mag()
				qc := to * s * math.Pi / 180
				th2, th4 := rockerFromCoupler(qc)
				crank, err := p.Pose.Values[0].In(units.Radian)
				require.NoError(t, err)
				follower, err := p.Pose.Values[2].In(units.Radian)
				require.NoError(t, err)
				require.InDelta(t, th2, crank, 1e-9, "crank at s = %v", s)
				require.InDelta(t, th4-t40, follower, 1e-9, "follower at s = %v", s)
				require.InDelta(t, to*s, p.Pose.Values[1].Mag(), 1e-12, "the coupler's stated value")
				for _, k := range []int{0, 2} {
					bound, err := p.Pose.Bounds[k].In(units.Radian)
					require.NoError(t, err)
					require.Greater(t, bound, 0.0)
					require.Less(t, bound, 1e-9)
				}
				got, err := sched.PoseAt(t.Context(), p.Pose.At)
				require.NoError(t, err)
				require.Equal(t, p.Pose, got)
			}
		})
	}
}

// TestVerifyJointBoxLoopOverTheCoupler is the joint box over the same loop
// axis (docs/linkage-check-design.md §16.1): the coupler over [0°, 30°], its
// parent the crank. Every leaf's centre reads the crank and the follower at
// the closed forms of the coupler's value there, within 1e-9 rad, and the box
// reads Sound.
func TestVerifyJointBoxLoopOverTheCoupler(t *testing.T) {
	t.Parallel()
	fb, _ := buildRocker(t, false)
	report, err := fb.doc.VerifyJointBox(t.Context(), fb.linkage, decad.JointBox{
		{Link: fb.couplerLk, Min: units.Degrees(0), Max: units.Degrees(30)},
	})
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
	require.NotEmpty(t, report.Cells)
	_, t40 := rockerFromCoupler(0)
	for _, cell := range report.Cells {
		qc, err := cell.Center.Values[1].In(units.Radian)
		require.NoError(t, err)
		th2, th4 := rockerFromCoupler(qc)
		crank, err := cell.Center.Values[0].In(units.Radian)
		require.NoError(t, err)
		follower, err := cell.Center.Values[2].In(units.Radian)
		require.NoError(t, err)
		require.InDelta(t, th2, crank, 1e-9)
		require.InDelta(t, th4-t40, follower, 1e-9)
	}
}
