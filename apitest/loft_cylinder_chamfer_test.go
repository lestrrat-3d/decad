package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestLoftCylinderChamfersBothOuterCaps chamfers both outer cap loops of one
// twisted Loft tooth joined to a circular blank. The result is a closed,
// verified shell whose vertices lie only on the cap planes and the two side
// levels one setback inside them. Each inset cap pulls the 7 mm tooth tip in
// by the setback, each bevel band rises from that inset to the side ring,
// and the removed volume matches half a setback times the cap area cut away.
func TestLoftCylinderChamfersBothOuterCaps(t *testing.T) {
	t.Parallel()
	const (
		height  = 1.0
		setback = 0.05
		tip     = 7.0
	)
	doc := decad.New()
	world := sketch.NewWorld()
	topFrame, err := r3.NewFrame(r3.Vec{Z: height}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	topPlane, err := world.CreatePlaneFromFrame(topFrame)
	require.NoError(t, err)
	bottomSketch, bottomProfile := loftMirrorChamferSection(t, world, world.XY(), 0)
	topSketch, topProfile := loftMirrorChamferSection(t, world, topPlane, 0.02)
	tooth, err := decad.Loft(t.Context(), doc, bottomSketch, bottomProfile, topSketch, topProfile)
	require.NoError(t, err)
	blank := circleBodyAtZ(t, doc, 0, 4, 0, height)
	joined, err := decad.Union(t.Context(), blank, tooth)
	require.NoError(t, err)

	capFaces := func(b *decad.Body) [2]*decad.Face {
		t.Helper()
		var caps [2]*decad.Face
		for i, role := range []decad.FeatureRef{decad.CapStart(b), decad.CapEnd(b)} {
			faces, err := decad.Faces(decad.FaceCreatedBy(role)).Exactly(1).SelectFaces(b)
			require.NoError(t, err)
			caps[i] = faces[0]
		}
		return caps
	}
	capAreas := func(b *decad.Body) float64 {
		t.Helper()
		total := 0.0
		for _, face := range capFaces(b) {
			area, err := face.Area()
			require.NoError(t, err)
			total += area.Value.Base()
		}
		return total
	}
	sourceArea := capAreas(joined)
	sourceVolume, err := joined.Volume()
	require.NoError(t, err)

	outerCaps := decad.Edges(decad.OuterLoopOf(decad.CapStart(joined))).
		Or(decad.OuterLoopOf(decad.CapEnd(joined)))
	chamfered, err := decad.Chamfer(t.Context(), joined, outerCaps, units.Millimeters(setback))
	require.NoError(t, err)

	mesh, err := chamfered.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Equal(t, 2, loftMirrorChamferEuler(mesh), "the chamfered shell must close like a sphere")

	// capLevel[i] is cap i's plane and sideLevel[i] its side ring's level.
	capLevel := [2]float64{0, height}
	sideLevel := [2]float64{setback, height - setback}
	onLevel := func(z float64) bool {
		for _, level := range append(capLevel[:], sideLevel[:]...) {
			if math.Abs(z-level) <= 1e-12 {
				return true
			}
		}
		return false
	}
	caps := capFaces(chamfered)
	bands := [2]*decad.Face{faceWithRole(t, chamfered, "chamferCap(0,0,0)"),
		faceWithRole(t, chamfered, "chamferCap(1,0,0)")}
	vertices, sources := mesh.Vertices(), mesh.SourceFaces()
	var capReach, bandCapReach, bandSideReach [2]float64
	for i, tri := range mesh.Triangles() {
		for _, index := range tri {
			p := vertices[index]
			require.True(t, onLevel(p.Z), "vertex z=%g is off the cap and side levels", p.Z)
			radius := math.Hypot(p.X, p.Y)
			for c := range 2 {
				switch sources[i] {
				case caps[c]:
					require.InDelta(t, capLevel[c], p.Z, 1e-12, "cap %d vertex leaves its plane", c)
					capReach[c] = max(capReach[c], radius)
				case bands[c]:
					switch {
					case math.Abs(p.Z-capLevel[c]) <= 1e-12:
						bandCapReach[c] = max(bandCapReach[c], radius)
					case math.Abs(p.Z-sideLevel[c]) <= 1e-12:
						bandSideReach[c] = max(bandSideReach[c], radius)
					default:
						require.Failf(t, "band vertex off its levels", "cap %d band vertex z=%g", c, p.Z)
					}
				}
			}
		}
	}
	for c := range 2 {
		// The inset tooth tip is the 7 mm tip arc moved in by the setback; the
		// band's cap edge is that inset and its side edge reaches the old wall,
		// whose held chords sit just inside the tip arc.
		require.InDelta(t, tip-setback, capReach[c], 1e-5, "cap %d inset tip", c)
		require.InDelta(t, capReach[c], bandCapReach[c], 1e-12, "cap %d band must meet its inset", c)
		require.InDelta(t, tip, bandSideReach[c], 1e-3, "cap %d band must reach the side ring", c)
		require.Greater(t, bandSideReach[c], capReach[c]+0.9*setback, "cap %d band must slope outward", c)
	}

	// Each 45-degree band cuts a triangle of legs d from the old edge, so the
	// removed volume is d/2 times the cap area the two insets give up.
	volume, err := chamfered.Volume()
	require.NoError(t, err)
	removed := sourceVolume.Value.Base() - volume.Value.Base()
	shrink := sourceArea - capAreas(chamfered)
	require.Positive(t, shrink)
	require.InEpsilon(t, setback/2*shrink, removed, 0.02)
}
