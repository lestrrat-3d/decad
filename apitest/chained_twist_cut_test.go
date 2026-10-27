package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad"
)

// The screw gear sleeve: a tube bored along two crossing axes by twisted
// rectangular tools, then cut by two slanted prism windows. The sizes below
// are the gear's; chainedBoreAxis names each tool's pose.
const (
	chainTubeOuter  = 18.5
	chainTubeInner  = 12.5
	chainTubeHeight = 37.5
	chainHalfWidth  = 7.7   // half of the 15.4 mm tool side
	chainHalfThick  = 2.075 // half of the 4.15 mm tool side
	chainLead       = 49.5  // axial advance per turn of the twist
	chainAxisLift   = 6.925 // each axis sits this far from mid height
	chainAxisAngle  = 40.0  // degrees about the tube axis
)

// chainStations are the two spans of each axis the bore tools cover.
var chainStations = [2][2]float64{{8.6, 19.5}, {-19.5, -8.6}}

// chainBoreSpec is one twisted tool: the axis it follows and its station span.
type chainBoreSpec struct {
	sign   float64 // +1 or -1: the axis, its lift and its angle
	s0, s1 float64 // station span along the axis, mm from the tube axis
}

func chainBoreSpecs() []chainBoreSpec {
	var out []chainBoreSpec
	for _, sign := range []float64{1, -1} {
		for _, st := range chainStations {
			out = append(out, chainBoreSpec{sign: sign, s0: st[0], s1: st[1]})
		}
	}
	return out
}

// pose returns the axis direction, the axial unit (the tool's local X) and
// the axis point at station zero.
func (c chainBoreSpec) pose() (dir, axial, origin r3.Vec) {
	ang := c.sign * chainAxisAngle * math.Pi / 180
	dir = r3.NewVec(math.Cos(ang), math.Sin(ang), 0)
	axial = r3.NewVec(0, 0, c.sign)
	origin = r3.NewVec(0, 0, -c.sign*chainAxisLift)
	return dir, axial, origin
}

func chainCross(a, b r3.Vec) r3.Vec {
	return r3.NewVec(a.Y*b.Z-a.Z*b.Y, a.Z*b.X-a.X*b.Z, a.X*b.Y-a.Y*b.X)
}

// chainTube is the 37.5 mm tube about the Z axis, centred on z = 0.
func chainTube(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	tube := twistBoreTube(t, doc, chainTubeOuter, chainTubeInner, chainTubeHeight)
	move, err := r3.Translation(r3.NewVec(0, 0, -chainTubeHeight/2))
	require.NoError(t, err)
	tube, err = decad.Transform(t.Context(), tube, move)
	require.NoError(t, err)
	return tube
}

// chainBoreTool builds one twisted tool in place. The rectangle is turned by
// the twist angle of its first station so the tool follows the gear's lead.
func chainBoreTool(t *testing.T, doc *decad.Document, c chainBoreSpec) *decad.Body {
	t.Helper()
	lam := chainLead / (2 * math.Pi)
	theta0 := c.s0 / lam
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	var pts []*sketch.Point
	for _, q := range [][2]float64{{-chainHalfWidth, -chainHalfThick}, {chainHalfWidth, -chainHalfThick},
		{chainHalfWidth, chainHalfThick}, {-chainHalfWidth, chainHalfThick}} {
		x := q[0]*math.Cos(theta0) - q[1]*math.Sin(theta0)
		y := q[0]*math.Sin(theta0) + q[1]*math.Cos(theta0)
		p := s.CreatePoint(x, y)
		s.Fix(p)
		pts = append(pts, p)
	}
	for i := range pts {
		s.CreateLine(pts[i], pts[(i+1)%4])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	path, err := decad.NewPath(r3.Vec{}, decad.LineTo{End: r3.NewVec(0, 0, c.s1-c.s0)})
	require.NoError(t, err)
	tool, err := decad.Sweep(t.Context(), doc, s, s.Profiles()[0], path,
		decad.WithSweepTwist(units.Radians((c.s1-c.s0)/lam)))
	require.NoError(t, err)
	dir, axial, origin := c.pose()
	f, err := r3.NewFrame(origin.Add(dir.Scale(c.s0)), axial, chainCross(dir, axial))
	require.NoError(t, err)
	place, err := r3.FromFrame(f)
	require.NoError(t, err)
	tool, err = decad.Transform(t.Context(), tool, place)
	require.NoError(t, err)
	return tool
}

// chainWindowSection holds the section corners in the XZ plane, slanted so the
// prism's side walls lean. The prism runs from the tube axis outward. This
// narrow window lies in the angular gap the four bores leave on the tube.
var chainWindowSection = [4][2]float64{{-1, -5}, {1, -5}, {2, 5}, {0, 5}}

// chainWideWindowSection is a window wide enough to reach the bores' rims.
var chainWideWindowSection = [4][2]float64{{-4, -5}, {4, -5}, {6, 5}, {-2, 5}}

// chainWindow builds the slanted window prism on the -Y side of the tube,
// turned by turn degrees about Z.
func chainWindow(t *testing.T, doc *decad.Document, section [4][2]float64, turn float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XZ())
	require.NoError(t, err)
	var pts []*sketch.Point
	for _, q := range section {
		p := s.CreatePoint(q[0], q[1])
		s.Fix(p)
		pts = append(pts, p)
	}
	for i := range pts {
		s.CreateLine(pts[i], pts[(i+1)%4])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(30), Dir: decad.Along})
	require.NoError(t, err)
	if turn == 0 {
		return body
	}
	rot, err := r3.Rotation(r3.NewVec(0, 0, 1), units.Degrees(turn))
	require.NoError(t, err)
	body, err = decad.Transform(t.Context(), body, rot)
	require.NoError(t, err)
	return body
}

// chainInTool reports whether the world point lies inside the twisted tool.
func chainInTool(c chainBoreSpec, x, y, z float64) bool {
	dir, axial, origin := c.pose()
	rel := r3.NewVec(x, y, z).Sub(origin)
	s := rel.Dot(dir)
	if s < c.s0 || s > c.s1 {
		return false
	}
	lam := chainLead / (2 * math.Pi)
	sin, cos := math.Sincos(s / lam)
	v := chainCross(dir, axial)
	a, b := rel.Dot(axial), rel.Dot(v)
	// Undo the turn: the section's local u is the axial coordinate.
	u := a*cos + b*sin
	w := -a*sin + b*cos
	return math.Abs(u) <= chainHalfWidth && math.Abs(w) <= chainHalfThick
}

// chainBoreVolume integrates, independently of the evaluator, the volume of
// the union of the bore tools inside the tube. It samples each tool's own
// coordinates on a midpoint grid and counts a sample once, for the first tool
// that holds it.
func chainBoreVolume() float64 {
	const nu, nv, ns = 308, 83, 218
	specs := chainBoreSpecs()
	lam := chainLead / (2 * math.Pi)
	total := 0.0
	for k, c := range specs {
		dir, axial, origin := c.pose()
		v := chainCross(dir, axial)
		du, dv, ds := 2*chainHalfWidth/nu, 2*chainHalfThick/nv, (c.s1-c.s0)/ns
		count := 0
		for m := range ns {
			s := c.s0 + (float64(m)+0.5)*ds
			sin, cos := math.Sincos(s / lam)
			for i := range nu {
				u := -chainHalfWidth + (float64(i)+0.5)*du
				for j := range nv {
					w := -chainHalfThick + (float64(j)+0.5)*dv
					p := origin.Add(dir.Scale(s)).Add(axial.Scale(u*cos - w*sin)).Add(v.Scale(u*sin + w*cos))
					if r2 := p.X*p.X + p.Y*p.Y; r2 < chainTubeInner*chainTubeInner || r2 > chainTubeOuter*chainTubeOuter {
						continue
					}
					if math.Abs(p.Z) > chainTubeHeight/2 {
						continue
					}
					shared := false
					for _, o := range specs[:k] {
						if chainInTool(o, p.X, p.Y, p.Z) {
							shared = true
							break
						}
					}
					if !shared {
						count++
					}
				}
			}
		}
		total += float64(count) * du * dv * ds
	}
	return total
}

// chainWindowVolume integrates, independently of the evaluator, the part of
// one window prism that lies in the tube and outside every bore tool. The
// prism runs along -Y from the tube axis, turned by turn degrees about Z. Each
// section point contributes the annulus's own y-range, sampled by midpoints.
func chainWindowVolume(section [4][2]float64, turn float64) float64 {
	const nu, nz, ny = 200, 200, 80
	minU, maxU, minZ, maxZ := math.Inf(1), math.Inf(-1), math.Inf(1), math.Inf(-1)
	for _, q := range section {
		minU, maxU = math.Min(minU, q[0]), math.Max(maxU, q[0])
		minZ, maxZ = math.Min(minZ, q[1]), math.Max(maxZ, q[1])
	}
	sin, cos := math.Sincos(turn * math.Pi / 180)
	du, dz := (maxU-minU)/nu, (maxZ-minZ)/nz
	specs := chainBoreSpecs()
	total := 0.0
	for i := range nu {
		u := minU + (float64(i)+0.5)*du
		lo := math.Sqrt(math.Max(chainTubeInner*chainTubeInner-u*u, 0))
		hi := math.Sqrt(chainTubeOuter*chainTubeOuter - u*u)
		dy := (hi - lo) / ny
		for j := range nz {
			z := minZ + (float64(j)+0.5)*dz
			if !chainInPolygon(section, u, z) {
				continue
			}
			for k := range ny {
				y := -(lo + (float64(k)+0.5)*dy)
				x, yy := u*cos-y*sin, u*sin+y*cos
				inBore := false
				for _, c := range specs {
					if chainInTool(c, x, yy, z) {
						inBore = true
						break
					}
				}
				if !inBore {
					total += du * dz * dy
				}
			}
		}
	}
	return total
}

// chainInPolygon is the even-odd test against the window section.
func chainInPolygon(section [4][2]float64, u, z float64) bool {
	in := false
	n := len(section)
	for i := range n {
		a, b := section[i], section[(i+1)%n]
		if (a[1] > z) == (b[1] > z) {
			continue
		}
		if u < a[0]+(z-a[1])*(b[0]-a[0])/(b[1]-a[1]) {
			in = !in
		}
	}
	return in
}

// chainBored cuts the four bore tools from the tube in turn and returns the
// bored body.
func chainBored(t *testing.T, doc *decad.Document, tube *decad.Body) *decad.Body {
	t.Helper()
	body := tube
	for i, c := range chainBoreSpecs() {
		var err error
		body, err = decad.Cut(t.Context(), body, chainBoreTool(t, doc, c))
		require.NoError(t, err, "bore cut %d", i)
		require.True(t, body.IsSolid(), "bore cut %d", i)
	}
	return body
}

// chainRequireVolume checks the body's volume against the tube minus the
// independently integrated bores and windows, and that it tessellates with
// every verification passing. It returns the volume's certified bound.
func chainRequireVolume(t *testing.T, body *decad.Body, tubeVolume decad.Measurement,
	windows float64) (float64, float64) {
	t.Helper()
	bores := chainBoreVolume()
	require.Greater(t, bores, 1000.0)
	want := tubeVolume.Value.Base() - bores - windows
	volume, err := body.Volume()
	require.NoError(t, err)
	// 0.5% of the removed volume covers the independent integral's sampling.
	slack := volume.Bound.Base() + tubeVolume.Bound.Base() + 0.005*(bores+windows)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-want), slack,
		"volume %v want %v (bores %v windows %v)", volume, want, bores, windows)

	mesh, err := body.Tessellate(t.Context(), units.Millimeters(3), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	return volume.Bound.Base(), mesh.Bound().Base()
}

// Four twisted bore cuts and two slanted prism window cuts chain on one tube.
// Every cut is admitted, the result is a solid that verifies, and its volume
// encloses the tube minus an independently integrated overlap.
func TestCutChainedTwistedBoresAndWindows(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	tube := chainTube(t, doc)
	tubeVolume, err := tube.Volume()
	require.NoError(t, err)

	body := chainBored(t, doc, tube)
	windows := 0.0
	for i, turn := range []float64{0, 180} {
		body, err = decad.Cut(t.Context(), body, chainWindow(t, doc, chainWindowSection, turn))
		require.NoError(t, err, "window cut %d", i)
		require.True(t, body.IsSolid(), "window cut %d", i)
		windows += chainWindowVolume(chainWindowSection, turn)
	}
	require.Greater(t, windows, 200.0)

	volumeBound, meshBound := chainRequireVolume(t, body, tubeVolume, windows)
	// The gallery proves 0.2 mm bore clearances against this body, so both
	// certified figures have to sit well inside that.
	require.Less(t, meshBound, 0.1)
	require.Less(t, volumeBound, 0.01*tubeVolume.Value.Base())
	t.Logf("volume bound %.2f mm3, boundary bound %.4f mm", volumeBound, meshBound)
}

// A window wide enough to reach the bores' rims meets facets that earlier
// cuts left coarser than the window pair's chord tolerance. The default
// refuses it, and the document's chain tolerance admits it with every bound
// still proven.
func TestCutChainedWindowOnBoreRims(t *testing.T) {
	t.Parallel()
	windowVolume := chainWindowVolume(chainWideWindowSection, 0)
	require.Greater(t, windowVolume, 400.0)

	doc := decad.New()
	bored := chainBored(t, doc, chainTube(t, doc))
	_, err := decad.Cut(t.Context(), bored, chainWindow(t, doc, chainWideWindowSection, 0))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "holds its mesh")

	doc = decad.New(decad.WithChainTolerance(units.Millimeters(0.1)))
	tube := chainTube(t, doc)
	tubeVolume, err := tube.Volume()
	require.NoError(t, err)
	body, err := decad.Cut(t.Context(), chainBored(t, doc, tube), chainWindow(t, doc, chainWideWindowSection, 0))
	require.NoError(t, err)
	require.True(t, body.IsSolid())
	chainRequireVolume(t, body, tubeVolume, windowVolume)
}
