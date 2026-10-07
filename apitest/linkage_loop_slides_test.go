package apitest_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// slideLoop is a closed loop with slides anywhere (docs/linkage-check-design.md
// §15.1-§15.2): its linkage, its links in Links() order, which of them slide,
// and the closed form of every joint value in Links() order — radians for a
// revolute, millimetres for a slide — given the listed link's value q.
type slideLoop struct {
	doc    *decad.Document
	l      *decad.Linkage
	links  []*decad.Link
	slides []bool
	closed func(listed int, q float64) []float64
}

// slideLayer is a small block on its own z layer near (x, y), so the layer
// exclusion settles every link pair and the drives read Sound.
func slideLayer(t *testing.T, doc *decad.Document, x, y float64, layer int) []*decad.Body {
	t.Helper()
	return []*decad.Body{boxBodyAtZ(t, doc, x-2, y-2, x+2, y+2, float64(10*layer), 8)}
}

// trammel is the elliptic trammel: a bar of length 50 whose end A slides
// along X and whose end B slides along Y, A at (40, 0) and B at (0, 30) at
// the zero pose. Links: the X slide (primary), the bar on it, the Y slide
// (anchored on Common).
func trammel(t *testing.T) slideLoop {
	t.Helper()
	z := r3.NewVec(0, 0, 1)
	doc := decad.New()
	l := decad.NewLinkage()
	sx, err := l.Ground().Prismatic(r3.NewVec(1, 0, 0), slideLayer(t, doc, 40, 0, 0))
	require.NoError(t, err)
	bar, err := sx.Revolute(r3.NewVec(40, 0, 0), z, slideLayer(t, doc, 20, 15, 1))
	require.NoError(t, err)
	sy, err := l.Ground().Prismatic(r3.NewVec(0, 1, 0), slideLayer(t, doc, 0, 30, 2))
	require.NoError(t, err)
	_, err = l.Close(bar, sy, r3.NewVec(0, 30, 0), z)
	require.NoError(t, err)
	alpha0 := math.Atan2(30, -40)
	at := func(x, y float64) []float64 {
		return []float64{x - 40, math.Atan2(y, -x) - alpha0, y - 30}
	}
	return slideLoop{doc: doc, l: l, links: []*decad.Link{sx, bar, sy}, slides: []bool{true, false, true},
		closed: func(listed int, q float64) []float64 {
			switch listed {
			case 0:
				x := 40 + q
				return at(x, math.Sqrt(2500-x*x))
			case 1:
				return at(-50*math.Cos(alpha0+q), 50*math.Sin(alpha0+q))
			}
			y := 30 + q
			return at(math.Sqrt(2500-y*y), y)
		}}
}

// rockingBlock is a block sliding along dir on a crank about the origin, its
// pin at B0 = (54, 32) on a rocker of length 40 about O4 = (30, 0), longer
// than the ground, so the crank turns freely. Links: the crank, the block on
// it (anchored on a revolute parent), the rocker. Along (27, 16) the rail
// passes through the crank's pivot; along (2, 1) it misses it by 2√5.
func rockingBlock(t *testing.T, dir r3.Vec) slideLoop {
	t.Helper()
	z := r3.NewVec(0, 0, 1)
	doc := decad.New()
	l := decad.NewLinkage()
	crank, err := l.Ground().Revolute(r3.Vec{}, z, slideLayer(t, doc, 10, 5, 0))
	require.NoError(t, err)
	block, err := crank.Prismatic(dir, slideLayer(t, doc, 54, 32, 1))
	require.NoError(t, err)
	rocker, err := l.Ground().Revolute(r3.NewVec(30, 0, 0), z, slideLayer(t, doc, 40, 15, 2))
	require.NoError(t, err)
	_, err = l.Close(block, rocker, r3.NewVec(54, 32, 0), z)
	require.NoError(t, err)
	n := math.Hypot(dir.X, dir.Y)
	dx, dy := dir.X/n, dir.Y/n
	t0 := 54*dx + 32*dy
	fx, fy := 54-t0*dx, 32-t0*dy
	e2 := fx*fx + fy*fy
	beta0 := math.Atan2(32, 24)
	// values reads the configuration at the pin B and the block's position t
	// along its rail.
	values := func(bx, by, tt float64) []float64 {
		th := math.Atan2(by, bx) - math.Atan2(fy+tt*dy, fx+tt*dx)
		return []float64{th, tt - t0, math.Atan2(by, bx-30) - beta0}
	}
	return slideLoop{doc: doc, l: l, links: []*decad.Link{crank, block, rocker}, slides: []bool{false, true, false},
		closed: func(listed int, q float64) []float64 {
			switch listed {
			case 2:
				bx, by := 30+40*math.Cos(beta0+q), 40*math.Sin(beta0+q)
				return values(bx, by, math.Sqrt(bx*bx+by*by-e2))
			case 1:
				tt := t0 + q
				r := math.Sqrt(e2 + tt*tt)
				bx := (r*r - 700) / 60
				return values(bx, math.Sqrt(r*r-bx*bx), tt)
			}
			// The crank turned by q: the rail is F + t·d rotated by q; its
			// point on the rocker's circle nearest t0 is the pin.
			c, s := math.Cos(q), math.Sin(q)
			px, py := c*fx-s*fy, s*fx+c*fy
			ux, uy := c*dx-s*dy, s*dx+c*dy
			// |P + t·U − O4|² = 1600: t² + 2t·U·(P − O4) + |P − O4|² − 1600 = 0.
			wx, wy := px-30, py
			b := ux*wx + uy*wy
			disc := math.Sqrt(b*b - (wx*wx + wy*wy - 1600))
			tt := -b + disc
			if math.Abs(-b-disc-t0) < math.Abs(tt-t0) {
				tt = -b - disc
			}
			return values(px+tt*ux, py+tt*uy, tt)
		}}
}

// scotchYoke is a crank of radius 30 about the origin, its pin at (18, 24),
// driving a yoke along X through a block that slides along Y on the yoke.
// Links: the yoke (primary), the block on it (anchored on a slide parent), the
// crank.
func scotchYoke(t *testing.T) slideLoop {
	t.Helper()
	z := r3.NewVec(0, 0, 1)
	doc := decad.New()
	l := decad.NewLinkage()
	yoke, err := l.Ground().Prismatic(r3.NewVec(1, 0, 0), slideLayer(t, doc, 18, 0, 0))
	require.NoError(t, err)
	block, err := yoke.Prismatic(r3.NewVec(0, 1, 0), slideLayer(t, doc, 18, 24, 1))
	require.NoError(t, err)
	crank, err := l.Ground().Revolute(r3.Vec{}, z, slideLayer(t, doc, 9, 12, 2))
	require.NoError(t, err)
	_, err = l.Close(block, crank, r3.NewVec(18, 24, 0), z)
	require.NoError(t, err)
	th0 := math.Atan2(24, 18)
	at := func(x, y float64) []float64 { return []float64{x - 18, y - 24, math.Atan2(y, x) - th0} }
	return slideLoop{doc: doc, l: l, links: []*decad.Link{yoke, block, crank}, slides: []bool{true, true, false},
		closed: func(listed int, q float64) []float64 {
			switch listed {
			case 0:
				x := 18 + q
				return at(x, math.Sqrt(900-x*x))
			case 1:
				y := 24 + q
				return at(math.Sqrt(900-y*y), y)
			}
			return at(30*math.Cos(th0+q), 30*math.Sin(th0+q))
		}}
}

// requireSlideLoopDrive drives lp's listed link 0 → to and asserts Sound, every
// dependent value within 1e-9 of the closed form with a positive bound below
// 1e-9, the listed value as stated, and a schedule posing every evaluated
// pose.
func requireSlideLoopDrive(t *testing.T, lp slideLoop, listed int, to float64) {
	t.Helper()
	unit, toward := units.Radian, units.New(to, units.Radian)
	if lp.slides[listed] {
		unit, toward = units.Millimeter, units.Millimeters(to)
	}
	drive := decad.Drive{{Link: lp.links[listed], From: units.New(0, unit), To: toward}}
	report := verifyLinkage(t, lp.doc, lp.l, drive)
	require.Equal(t, decad.Sound, report.Status, "%+v", report.Diagnostics)
	require.NotEmpty(t, report.Poses)
	sched, err := lp.l.Schedule(t.Context(), drive)
	require.NoError(t, err)
	for _, p := range report.Poses {
		s := p.Pose.At.Mag()
		want := lp.closed(listed, to*s)
		for k := range lp.links {
			u := units.Radian
			if lp.slides[k] {
				u = units.Millimeter
			}
			got, err := p.Pose.Values[k].In(u)
			require.NoError(t, err)
			require.InDelta(t, want[k], got, 1e-9, "link %d at s = %v", k, s)
			if k == listed {
				continue
			}
			bound, err := p.Pose.Bounds[k].In(u)
			require.NoError(t, err)
			require.Greater(t, bound, 0.0)
			require.Less(t, bound, 1e-9)
		}
		got, err := sched.PoseAt(t.Context(), p.Pose.At)
		require.NoError(t, err)
		require.Equal(t, p.Pose, got)
	}
}

// requireSlideLoopDrives drives every joint of the loop build returns to its
// entry of to, both ways, through requireSlideLoopDrive.
func requireSlideLoopDrives(t *testing.T, build func(t *testing.T) slideLoop, to []float64) {
	t.Helper()
	for listed, far := range to {
		for _, sign := range []float64{1, -1} {
			t.Run(fmt.Sprintf("link %d to %+.4g", listed, sign*far), func(t *testing.T) {
				t.Parallel()
				requireSlideLoopDrive(t, build(t), listed, sign*far)
			})
		}
	}
}

// TestVerifyLinkageLoopTrammel pins docs/linkage-check-design.md §15.2's
// anchored rail on Common: the elliptic trammel, two slides on Common, the
// second anchored on fixed points, driven at each slide and at the bar, a
// revolute driver below the primary slide, both ways. Every pose matches the
// closed forms.
//
// Legs seen to fail when deleted: the anchor ahead of a backward slide driver
// (the slide along Y driven −12 mm is refused), and the anchored dependent's
// sense +1 (the slide along Y reads its displacement negated).
func TestVerifyLinkageLoopTrammel(t *testing.T) {
	t.Parallel()
	requireSlideLoopDrives(t, trammel, []float64{8, 20 * math.Pi / 180, 12})
}

// TestVerifyLinkageLoopRockingBlock pins the anchored rail on a turning
// crank, its line through the crank's pivot: the rocking block driven at the
// crank, the block and the rocker, both ways. Every pose matches the closed
// forms.
//
// Legs seen to fail when deleted: the anchor ahead of a backward slide driver
// (the block driven −5 mm is refused), and any one of the five distances that
// hold the rail to the crank's frame (the rail turns or slides free and E0
// refuses).
func TestVerifyLinkageLoopRockingBlock(t *testing.T) {
	t.Parallel()
	requireSlideLoopDrives(t, func(t *testing.T) slideLoop { return rockingBlock(t, r3.NewVec(27, 16, 0)) }, rockingBlockTravel)
}

// TestVerifyLinkageLoopRockingBlockOffset is TestVerifyLinkageLoopRockingBlock
// with the rail off the crank's pivot by 2√5, its legs the same.
func TestVerifyLinkageLoopRockingBlockOffset(t *testing.T) {
	t.Parallel()
	requireSlideLoopDrives(t, func(t *testing.T) slideLoop { return rockingBlock(t, r3.NewVec(2, 1, 0)) }, rockingBlockTravel)
}

// rockingBlockTravel is the far end of each rocking-block drive: the crank,
// the block, the rocker.
var rockingBlockTravel = []float64{8 * math.Pi / 180, 3, 15 * math.Pi / 180}

// TestVerifyLinkageLoopScotchYoke pins the anchored rail on a slide: the
// Scotch yoke's block on the yoke, driven at the yoke, the block and the
// crank, both ways. Every pose matches the closed forms.
//
// Legs seen to fail when deleted: the anchor ahead of a backward slide driver
// (the block driven −4 mm is refused), and the distance ZA that holds the
// block's rail to the yoke (the rail turns free and E0 refuses).
func TestVerifyLinkageLoopScotchYoke(t *testing.T) {
	t.Parallel()
	requireSlideLoopDrives(t, scotchYoke, []float64{6, 4, 20 * math.Pi / 180})
}

// TestVerifyJointBoxLoopSlides is the joint box over an anchored slide
// (docs/linkage-check-design.md §16): the Scotch yoke's block over
// [−4, 4] mm, its parent the yoke. Every leaf's centre reads the yoke and the
// crank at the closed forms of the block's value there, within 1e-9, and
// the box reads Sound.
func TestVerifyJointBoxLoopSlides(t *testing.T) {
	t.Parallel()
	lp := scotchYoke(t)
	report, err := lp.doc.VerifyJointBox(t.Context(), lp.l, decad.JointBox{
		{Link: lp.links[1], Min: units.Millimeters(-4), Max: units.Millimeters(4)},
	})
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status, "%+v", report.Diagnostics)
	require.NotEmpty(t, report.Cells)
	for _, cell := range report.Cells {
		q, err := cell.Center.Values[1].In(units.Millimeter)
		require.NoError(t, err)
		want := lp.closed(1, q)
		yoke, err := cell.Center.Values[0].In(units.Millimeter)
		require.NoError(t, err)
		crank, err := cell.Center.Values[2].In(units.Radian)
		require.NoError(t, err)
		require.InDelta(t, want[0], yoke, 1e-9)
		require.InDelta(t, want[2], crank, 1e-9)
	}
}
