package brepmodify

import (
	"math"
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

const (
	probeD    = 0.5
	probeOver = 0.25
)

// peninsulaLoop is the keyhole's loop with a peninsula of material, between x = a and x = b, rising from the slot's floor
// at y = -8 to y = top. The slot's right half opens to the plate's edge, and the bore's arc is the run.
func peninsulaLoop(a, b, top float64) (*chamferLoop, chamferRun) {
	const r, w = 4.0, keyholeHalfSlot
	y0 := math.Sqrt(r*r - w*w)
	segs := []loopSeg{
		lineSeg(pt2{-10, -10}, pt2{b, -10}),
		lineSeg(pt2{b, -10}, pt2{b, top}),
		lineSeg(pt2{b, top}, pt2{a, top}),
		lineSeg(pt2{a, top}, pt2{a, -8}),
		lineSeg(pt2{a, -8}, pt2{-w, -8}),
		lineSeg(pt2{-w, -8}, pt2{-w, -y0}),
		arcSeg(pt2{}, r, pt2{-w, -y0}, pt2{w, -y0}, false),
		lineSeg(pt2{w, -y0}, pt2{w, -10}),
		lineSeg(pt2{w, -10}, pt2{10, -10}),
		lineSeg(pt2{10, -10}, pt2{10, 10}),
		lineSeg(pt2{10, 10}, pt2{-10, 10}),
		lineSeg(pt2{-10, 10}, pt2{-10, -10}),
	}
	l := testLoop(segs...)
	return l, chamferRun{loop: l, first: 6, count: 1}
}

// TestCheckExtensionIsExactForThinFeatures refuses a feature of any thickness inside a cutter's end, and admits one
// that stands outside it.
func TestCheckExtensionIsExactForThinFeatures(t *testing.T) {
	t.Parallel()
	reach := partialReach(probeD, probeOver)
	for _, c := range []struct {
		name      string
		a, b, top float64
		refuse    bool
	}{
		{"no peninsula in reach", 0.1, 0.12, -9, false},
		{"a peninsula 0.02 mm thick", 0.31, 0.33, -4.3, true},
		{"a peninsula 0.0001 mm thick", 0.3, 0.3001, -4.3, true},
		{"a peninsula below the cutter's reach", 0.31, 0.33, -6.5, false},
	} {
		t.Run(c.name, func(t *testing.T) {
			t.Parallel()
			_, run := peninsulaLoop(c.a, c.b, c.top)
			err := run.checkExtension(probeD, reach)
			if c.refuse {
				require.ErrorIs(t, err, decaderr.ErrUnsupported)
				return
			}
			require.NoError(t, err)
		})
	}
}

// A line run's strip past its end is tested the same way.
func TestCheckExtensionOfALineRunIsExactForThinTabs(t *testing.T) {
	t.Parallel()
	reach := partialReach(probeD, probeOver)
	for _, c := range []struct {
		name   string
		y0, y1 float64
		refuse bool
	}{
		{"a tab 0.03 mm thick", 0.26, 0.29, true},
		{"a tab 0.0001 mm thick", 0.26, 0.2601, true},
		{"a tab beyond the strip", 1.2, 1.3, false},
	} {
		t.Run(c.name, func(t *testing.T) {
			t.Parallel()
			l := testLoop(
				lineSeg(pt2{0, 0}, pt2{30, 0}),
				lineSeg(pt2{30, 0}, pt2{30, c.y0}), lineSeg(pt2{30, c.y0}, pt2{31, c.y0}),
				lineSeg(pt2{31, c.y0}, pt2{31, c.y1}), lineSeg(pt2{31, c.y1}, pt2{30, c.y1}),
				lineSeg(pt2{30, c.y1}, pt2{30, 10}), lineSeg(pt2{30, 10}, pt2{0, 10}), lineSeg(pt2{0, 10}, pt2{0, 0}))
			err := chamferRun{loop: l, first: 0, count: 1}.checkExtension(probeD, reach)
			if c.refuse {
				require.ErrorIs(t, err, decaderr.ErrUnsupported)
				return
			}
			require.NoError(t, err)
		})
	}
}

// smallConvexArcLoop is the loop of a plate whose convex arc, of radius r, two sharp corners turn by turn each.
func smallConvexArcLoop(r, turn float64) (*chamferLoop, chamferRun) {
	p1 := pt2{20, 0}
	c := pt2{p1.x - r*math.Sin(turn), p1.y + r*math.Cos(turn)}
	a1 := turn - math.Pi/2 + math.Pi/3
	p2 := pt2{c.x + r*math.Cos(a1), c.y + r*math.Sin(a1)}
	dir := pt2{math.Cos(a1 + math.Pi/2 + turn), math.Sin(a1 + math.Pi/2 + turn)}
	p3 := p2.add(dir.scale(8))
	l := testLoop(lineSeg(pt2{0, 0}, p1), arcSeg(c, r, p1, p2, true), lineSeg(p2, p3), lineSeg(p3, pt2{0, 12}),
		lineSeg(pt2{0, 12}, pt2{0, 0}))
	return l, chamferRun{loop: l, first: 1, count: 1}
}

// The bevel's circle at the cap has radius r - d, and a next wall at r cos(turn) from the centre is not reached by it
// (past it by the margin) when that is larger: the cutter's end face would lie in the wall's material.
func TestArcRunOutAngleRefusesABevelThatMissesTheNextWall(t *testing.T) {
	t.Parallel()
	deg := math.Pi / 180
	for _, c := range []struct {
		r, turn float64
		refuse  bool
	}{{1.2, 45, true}, {0.9, 60, true}, {6, 70, false}, {2, 70, false}, {1.5, 50, true}} {
		t.Run("", func(t *testing.T) {
			t.Parallel()
			l, run := smallConvexArcLoop(c.r, c.turn*deg)
			// The end face clears the next wall by the margin only if the bevel's circle at the cap reaches that far past it.
			require.Equal(t, c.refuse, c.r-probeD-c.r*math.Cos(c.turn*deg) < probeOver)
			ends, ok := run.ends()
			require.True(t, ok)
			for _, e := range ends {
				_, err := l.arcRunOutAngle(e, run.seg(0), probeD, probeOver)
				if c.refuse {
					require.ErrorIs(t, err, decaderr.ErrUnsupported)
					continue
				}
				require.NoError(t, err)
			}
		})
	}
}

// The run-out of a bore's arc stops the margin past the slot wall, on both ends alike: each end face clears its wall by
// the margin at both its radii, and a shorter turn would not.
func TestArcRunOutAngleStopsTheMarginPastTheWall(t *testing.T) {
	t.Parallel()
	for _, r := range []float64{2.55, 4, 4.05} {
		l := keyholeLoop(r, 0)
		run := chamferRun{loop: l, first: 2, count: 1}
		ends, ok := run.ends()
		require.True(t, ok)
		seg := run.seg(0)
		for i, e := range ends {
			alpha, err := l.arcRunOutAngle(e, seg, probeD, probeOver)
			require.NoError(t, err)
			// The wall is the vertical line through the end's corner, with the slot on the side of the axis.
			a0 := math.Atan2(e.v.y, e.v.x)
			slotward := -math.Copysign(1, e.v.x)
			side := func(theta float64) float64 {
				least := math.Inf(1)
				for _, rho := range []float64{r - probeOver, r + probeD} {
					p := pt2{rho * math.Cos(a0+theta), rho * math.Sin(a0+theta)}
					least = min(least, slotward*(p.x-e.v.x))
				}
				return least
			}
			require.GreaterOrEqual(t, side(alpha), probeOver-1e-9, "end %d", i)
			require.Less(t, side(0.99*alpha), probeOver, "end %d", i)
		}
	}
}

// TestClipRadiusStopsAtTheFootOfTheNextWall reads the end clip of an arc that meets a concave corner. The clip stops at
// the foot of the perpendicular from the arc's centre onto the next wall's line when that lies inside the sector, and
// keeps the sector's radius otherwise.
func TestClipRadiusStopsAtTheFootOfTheNextWall(t *testing.T) {
	t.Parallel()
	// An arc of radius R about the origin, counterclockwise with the material inside, ends at (0, R) and turns right by
	// the angle phi onto a line: a concave corner. The loop is closed well away from the corner.
	end := func(R, phi float64) runEnd {
		q := pt2{-30 * math.Cos(phi), R + 30*math.Sin(phi)}
		l := testLoop(
			arcSeg(pt2{}, R, pt2{R, 0}, pt2{0, R}, true),
			lineSeg(pt2{0, R}, q),
			lineSeg(q, pt2{-60, -60}),
			lineSeg(pt2{-60, -60}, pt2{R + 10, -60}),
			lineSeg(pt2{R + 10, -60}, pt2{R + 10, 0}),
			lineSeg(pt2{R + 10, 0}, pt2{R, 0}),
		)
		ends, _ := chamferRun{loop: l, first: 0, count: 1}.ends()
		require.True(t, ends[1].concave)
		return ends[1]
	}
	const d, over = 0.25, 0.25
	t.Run("foot inside the sector", func(t *testing.T) {
		t.Parallel()
		R, phi := 13.2, 11.43*math.Pi/180
		e := end(R, phi)
		require.InDelta(t, R*math.Sin(phi), e.foot, 1e-9)
		require.InDelta(t, R*math.Sin(phi), e.clipRadius(d, over), 1e-9)
	})
	t.Run("foot outside the sector", func(t *testing.T) {
		t.Parallel()
		R, phi := 13.95, 22.49*math.Pi/180
		e := end(R, phi)
		require.InDelta(t, R*math.Sin(phi), e.foot, 1e-9)
		sector := 1.5 * ((d+2*over)/math.Sin(phi) + d + over)
		require.Less(t, sector, e.foot)
		require.InDelta(t, sector, e.clipRadius(d, over), 1e-9)
	})
	t.Run("a line end keeps the sector's radius", func(t *testing.T) {
		t.Parallel()
		phi := 20 * math.Pi / 180
		e := runEnd{concave: true, turn: phi}
		require.InDelta(t, 1.5*((d+2*over)/math.Sin(phi)+d+over), e.clipRadius(d, over), 1e-9)
	})
}
