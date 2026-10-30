package saddlecanal

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/crossbore"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

var identity = crossbore.Pose{Axis: [3]int{0, 1, 2}, Sign: [3]float64{1, 1, 1}}

// exactPoint is F(θ, t) at any θ, by the formula of the package comment.
func exactPoint(c Canal, theta, t float64) r3.Vec {
	s, co := math.Sincos(theta)
	rp, Rp := c.Bore+c.Rho, c.R-c.Rho
	a1Sign := 1.0
	if c.Outside {
		Rp, a1Sign = c.R+c.Rho, -1
	}
	x := math.Sqrt(Rp*Rp - rp*rp*co*co)
	a1 := r3.Vec{X: x / Rp, Y: rp * co / Rp}.Scale(a1Sign)
	a2 := r3.Vec{Y: -co, Z: -s}
	b := a1.Add(a2)
	b = b.Scale(1 / b.Len())
	w := a1.Scale(1 - 2*t).Add(b.Scale(2 * t))
	if t > 0.5 {
		w = b.Scale(2 - 2*t).Add(a2.Scale(2*t - 1))
	}
	return r3.Vec{X: x, Y: rp * co, Z: rp * s}.Add(w.Scale(c.Rho / w.Len()))
}

// proofCanals are the convex mouth of a cross hole (inside the larger
// cylinder) and the two patterns outside it: H03's crossing bores in a block
// (R 5, r 3) and H08's boss on a rod (R 15, r 5).
var proofCanals = []Canal{
	{R: 10, Bore: 3, Rho: 0.5}, {R: 10, Bore: 3, Rho: 1.5}, {R: 10, Bore: 7, Rho: 0.5},
	{R: 5, Bore: 3, Rho: 0.5, Outside: true}, {R: 15, Bore: 5, Rho: 1, Outside: true},
}

func TestProveEnclosesTheFillet(t *testing.T) {
	t.Parallel()
	for i, c := range proofCanals {
		// Under the race detector one canal of each side runs.
		if raceDetector && i != 0 && i != len(proofCanals)-1 {
			continue
		}
		rows, err := c.Rows()
		require.NoError(t, err)
		p, err := Prove(t.Context(), c, rows, identity, func(k, j int) r3.Vec { return c.Point(k, rows[j]) })
		require.NoError(t, err)

		// The fillet's area by the midpoint rule on a grid 4 times finer
		// than the strip, central differences for the partials.
		const nt, nu = 4 * Stations, 128
		area := 0.0
		for i := range nt {
			theta := 2 * math.Pi * (float64(i) + 0.5) / nt
			for j := range nu {
				u := (float64(j) + 0.5) / nu
				const d = 1e-6
				ft := exactPoint(c, theta+d, u).Sub(exactPoint(c, theta-d, u)).Scale(1 / (2 * d))
				fu := exactPoint(c, theta, u+d).Sub(exactPoint(c, theta, u-d)).Scale(1 / (2 * d))
				area += ft.Cross(fu).Len() * 2 * math.Pi / nt / nu
			}
		}
		require.Less(t, p.AreaLo, area, "%+v: the area interval holds the fillet's area", c)
		require.Greater(t, p.AreaHi, area, "%+v: the area interval holds the fillet's area", c)

		// The walls' areas by Green's theorem in the unrolled shaft wall,
		// ∮ R·ψ dz along T1 and the seam, and along the bore's generators.
		rp, Rp := c.Offsets()
		shaft, bore := 0.0, 0.0
		for i := range nt {
			theta := 2 * math.Pi * (float64(i) + 0.5) / nt
			co := math.Cos(theta)
			shaft += (c.R*math.Asin(rp*co/Rp)*rp*co - c.R*math.Asin(c.Bore*co/c.R)*c.Bore*co) * 2 * math.Pi / nt
			bore += c.Bore * math.Abs(math.Sqrt(c.R*c.R-c.Bore*c.Bore*co*co)-math.Sqrt(Rp*Rp-rp*rp*co*co)) * 2 * math.Pi / nt
		}
		require.True(t, p.ShaftLo < shaft && shaft < p.ShaftHi, "%+v: the shaft wall's area %v in [%v, %v]",
			c, shaft, p.ShaftLo, p.ShaftHi)
		require.True(t, p.BoreLo < bore && bore < p.BoreHi, "%+v: the bore's area %v in [%v, %v]",
			c, bore, p.BoreLo, p.BoreHi)

		// No sampled point of a held strip triangle lies farther than the
		// chord bound from the fillet at its own parameters.
		worst := 0.0
		for k := 0; k < Stations; k += 7 {
			for j := range len(rows) - 1 {
				t0, t1 := rows[j], rows[j+1]
				th0, th1 := Theta(k), Theta(k+1)
				a, b, d := c.Point(k, t0), c.Point(k+1, t0), c.Point(k, t1)
				for _, w := range [][2]float64{{1.0 / 3, 1.0 / 3}, {0.5, 0}, {0, 0.5}, {0.5, 0.5}} {
					held := a.Add(b.Sub(a).Scale(w[0])).Add(d.Sub(a).Scale(w[1]))
					exact := exactPoint(c, th0+w[0]*(th1-th0), t0+w[1]*(t1-t0))
					worst = math.Max(worst, held.Sub(exact).Len())
				}
			}
		}
		require.Less(t, worst, p.Chord, "%+v: the chord bound covers the held strip", c)
	}
}

func TestProveReadsTheHeldVertices(t *testing.T) {
	t.Parallel()
	c := Canal{R: 10, Bore: 3, Rho: 0.5}
	rows, err := c.Rows()
	require.NoError(t, err)
	const moved = 1e-4
	p, err := Prove(t.Context(), c, rows, identity, func(k, j int) r3.Vec {
		v := c.Point(k, rows[j])
		if k == 17 && j == 3 {
			v.Z += moved
		}
		return v
	})
	require.NoError(t, err)
	require.GreaterOrEqual(t, p.Sample, moved, "a moved held vertex raises the sample error")
	require.Greater(t, p.Chord, p.Sample, "the chord bound carries the sample error")
}

func TestProveRefusesAFold(t *testing.T) {
	t.Parallel()
	// Near the size limit the ball's centre curve turns tighter than the
	// ball: the pipe folds at θ = 0, where c_x = √(R'² − r'²) is least.
	c := Canal{R: 10, Bore: 3, Rho: 3.4}
	require.True(t, c.Exists())
	rows, err := c.Rows()
	require.NoError(t, err)
	held := func(k, j int) r3.Vec { return c.Point(k, rows[j]) }
	_, err = Prove(t.Context(), c, rows, identity, held)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "folds: the pipe is not regular there")
}

func TestExists(t *testing.T) {
	t.Parallel()
	require.True(t, Canal{R: 10, Bore: 3, Rho: math.Nextafter(3.5, 0)}.Exists())
	require.False(t, Canal{R: 10, Bore: 3, Rho: 3.5}.Exists(), "r + ρ = R − ρ leaves no loop")
	require.False(t, Canal{R: 10, Bore: 3}.Exists())
	// Outside the larger cylinder r' = r + ρ < R + ρ = R' for every ball.
	require.True(t, Canal{R: 10, Bore: 3, Rho: 3.5, Outside: true}.Exists())
	require.True(t, Canal{R: 10, Bore: 3, Rho: 1e6, Outside: true}.Exists())
	require.False(t, Canal{R: 10, Bore: 3, Outside: true}.Exists())
	require.False(t, Canal{R: 3, Bore: 3, Rho: 1, Outside: true}.Exists(), "equal radii")
	_, err := Prove(t.Context(), Canal{R: 10, Bore: 3, Rho: 3.5}, []float64{0, 1}, identity, nil)
	require.ErrorIs(t, err, decaderr.ErrDegenerate)
	require.ErrorContains(t, err, "the saddle canal's offset cylinders do not meet in a loop")
}

func TestRowsSplitTheWidestArc(t *testing.T) {
	t.Parallel()
	for _, c := range proofCanals {
		rows, err := c.Rows()
		require.NoError(t, err)
		m := len(rows) - 1
		require.Zero(t, m%2)
		require.Equal(t, 0.5, rows[m/2])
		// Where the arc is widest, θ = 0 inside the larger cylinder and
		// θ = π/2 outside it, consecutive rows are no more than arcStep apart
		// in angle about the ball's centre.
		rp, Rp := c.Offsets()
		theta := 0.0
		if c.Outside {
			theta = math.Pi / 2
		}
		s, co := math.Sincos(theta)
		ball := r3.Vec{X: math.Sqrt(Rp*Rp - rp*rp*co*co), Y: rp * co, Z: rp * s}
		for j := range m {
			a := exactPoint(c, theta, rows[j]).Sub(ball)
			b := exactPoint(c, theta, rows[j+1]).Sub(ball)
			step := math.Acos(math.Min(1, a.Dot(b)/(a.Len()*b.Len())))
			require.LessOrEqual(t, step, arcStep*(1+1e-9), "%+v: row %d", c, j)
		}
	}
}
