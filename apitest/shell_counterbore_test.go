package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The plate, through bore, and counterbore all come from public sketch and
// feature calls. Either Cut order must leave the same rounded shell.
func TestShellCoaxialCounterbore(t *testing.T) {
	for _, centerX := range []float64{0, 2} {
		for _, fromTop := range []bool{true, false} {
			for _, throughFirst := range []bool{true, false} {
				name := "top"
				if !fromTop {
					name = "bottom"
				}
				if centerX != 0 {
					name += "/offset"
				}
				if !throughFirst {
					name += "/counterbore-first"
				} else {
					name += "/through-first"
				}
				t.Run(name, func(t *testing.T) {
					doc := decad.New()
					plate := boxBody(t, doc, -10, -10, 10, 10, 10)
					through := circleBodyAtZ(t, doc, centerX, 3, 0, 10)
					boreZ := 0.0
					if fromTop {
						boreZ = 7
					}
					wide := circleBodyAtZ(t, doc, centerX, 5, boreZ, 3)
					var part *decad.Body
					var err error
					if throughFirst {
						part, err = decad.Cut(t.Context(), plate, through)
						require.NoError(t, err)
						part, err = decad.Cut(t.Context(), part, wide)
					} else {
						part, err = decad.Cut(t.Context(), plate, wide)
						require.NoError(t, err)
						part, err = decad.Cut(t.Context(), part, through)
					}
					require.NoError(t, err)
					selectedCap := decad.CapStart(part)
					if fromTop {
						selectedCap = decad.CapEnd(part)
					}
					shell, err := decad.Shell(t.Context(), part, decad.Faces(decad.FaceCreatedBy(selectedCap)).Exactly(1),
						units.Millimeters(1))
					require.NoError(t, err)
					require.Len(t, shell.Lumps(), 1)
					torusFaces := 0
					for _, face := range shell.Faces() {
						if _, ok := face.Surface().(decad.Torus); ok {
							torusFaces++
						}
					}
					require.Equal(t, 1, torusFaces)
					volume, err := shell.Volume()
					require.NoError(t, err)
					want := 1084 + (227.0/3)*math.Pi + 2.5*math.Pi*math.Pi
					require.LessOrEqual(t, math.Abs(volume.Value.Base()-want), volume.Bound.Base()+1e-9)
					require.Less(t, volume.Bound.Base(), 1e-8)
					centroid, err := shell.Centroid()
					require.NoError(t, err)
					wantZ := (3962 + (6043.0/12)*math.Pi + 17.5*math.Pi*math.Pi) / want
					if !fromTop {
						wantZ = 10 - wantZ
					}
					wantX := centerX * (want - 1084) / want
					require.LessOrEqual(t, math.Abs(centroid.Value.X-wantX), centroid.Bound.Base()+1e-9)
					require.LessOrEqual(t, math.Abs(centroid.Value.Y), centroid.Bound.Base()+1e-9)
					require.LessOrEqual(t, math.Abs(centroid.Value.Z-wantZ), centroid.Bound.Base()+1e-9)
					require.Less(t, centroid.Bound.Base(), 1e-8)
					mesh, err := shell.Tessellate(t.Context(), units.Millimeters(0.1),
						decad.WithVerification(decad.VerifyAll))
					require.NoError(t, err)
					require.True(t, mesh.BoundaryVerified())
					require.True(t, mesh.VolumeVerified())
					require.Less(t, mesh.Bound().Base(), 0.1)
					requireWatertight(t, mesh)
					report, err := doc.Verify(t.Context())
					require.NoError(t, err)
					require.Equal(t, decad.Sound, report.Status)
					// The shoulder torus is convex: the solid lies inside its tube,
					// and it faces away from the axis. The tightest concave radius
					// is the kept small bore's 3 mm.
					report, err = doc.Verify(t.Context(), decad.WithConcaveRadius())
					require.NoError(t, err)
					require.Len(t, report.Bodies, 1)
					minimum := report.Bodies[0].ConcaveRadius.Minimum
					require.NotNil(t, minimum)
					require.Equal(t, decad.Exact, minimum.Exactness)
					require.True(t, minimum.Value.Equal(units.Millimeters(3), 1e-12),
						"the small bore's radius, got %s", minimum.Value)
				})
			}
		}
	}
}

func TestShellCounterboreRefusesInsufficientShoulder(t *testing.T) {
	doc := decad.New()
	plate := boxBody(t, doc, -10, -10, 10, 10, 10)
	through := circleBodyAtZ(t, doc, 0, 3, 0, 10)
	part, err := decad.Cut(t.Context(), plate, through)
	require.NoError(t, err)
	wide := circleBodyAtZ(t, doc, 0, 3.5, 7, 3)
	part, err = decad.Cut(t.Context(), part, wide)
	require.NoError(t, err)
	before := doc.Bodies()
	_, err = decad.Shell(t.Context(), part, decad.Faces(decad.FaceCreatedBy(decad.CapEnd(part))).Exactly(1),
		units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "insufficient shoulder")
	require.Equal(t, before, doc.Bodies())
}
