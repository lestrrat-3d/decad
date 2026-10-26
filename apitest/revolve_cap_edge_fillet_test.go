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

func rimQuarterCylinder(t *testing.T, xlo, xhi, radius, angle float64) (*decad.Document, *decad.Body) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(xlo, 0, xhi, radius)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	d := decad.New()
	b, err := decad.Revolve(t.Context(), d, s, s.Profiles()[0],
		decad.SketchLine{Start: decad.Point2{}, End: decad.Point2{U: 1}},
		decad.AngleExtent{A: units.Degrees(angle), Dir: decad.Along})
	require.NoError(t, err)
	return d, b
}

func quarterRim(xhi float64) decad.EdgeSelector {
	return decad.Edges(decad.ParallelTo(r3.NewVec(0, 1, 0)),
		decad.EndpointAt(r3.NewVec(xhi, 0, 0))).Exactly(1)
}

func TestRevolveQuarterCapRadialRimFilletVerified(t *testing.T) {
	for _, tc := range []struct {
		name                     string
		xlo, xhi, radius, fillet float64
	}{
		{name: "ten by eight", xlo: 0, xhi: 10, radius: 8, fillet: 1},
		{name: "shifted six by five", xlo: -2, xhi: 4, radius: 5, fillet: 0.5},
	} {
		t.Run(tc.name, func(t *testing.T) {
			doc, source := rimQuarterCylinder(t, tc.xlo, tc.xhi, tc.radius, 90)
			out, err := decad.Fillet(t.Context(), source, quarterRim(tc.xhi), units.Millimeters(tc.fillet))
			require.NoError(t, err)
			require.Equal(t, []*decad.Body{out}, doc.Bodies())
			require.Len(t, out.Faces(), 6)
			for _, endCap := range []decad.FeatureRef{decad.CapStart(source), decad.CapEnd(source)} {
				found, err := decad.Faces(decad.FaceCreatedBy(endCap)).Exactly(1).SelectFaces(out)
				require.NoError(t, err)
				require.Len(t, found, 1)
			}
			blendFaces := 0
			for _, face := range out.Faces() {
				for _, origin := range face.Origins() {
					if origin.Role == "fillet(0,1)" {
						blendFaces++
					}
				}
			}
			require.Equal(t, 1, blendFaces)
			mesh, err := out.Tessellate(t.Context(), units.Millimeters(0.1),
				decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			require.True(t, mesh.BoundaryVerified())
			require.True(t, mesh.VolumeVerified())
			require.Less(t, mesh.Bound().Mag(), 0.1)
			for _, corner := range []r3.Vec{
				r3.NewVec(tc.xhi-tc.fillet, 0, 0),
				r3.NewVec(tc.xhi-tc.fillet, tc.radius, 0),
				r3.NewVec(tc.xhi, 0, tc.fillet),
				r3.NewVec(tc.xhi, math.Sqrt(tc.radius*tc.radius-tc.fillet*tc.fillet), tc.fillet),
			} {
				nearest := math.Inf(1)
				for _, vertex := range mesh.Vertices() {
					nearest = math.Min(nearest, vertex.Sub(corner).Len())
				}
				require.LessOrEqual(t, nearest, mesh.Bound().Mag())
			}

			volume, err := out.Volume()
			require.NoError(t, err)
			base := (tc.xhi - tc.xlo) * math.Pi * tc.radius * tc.radius / 4
			spandrel := tc.fillet * tc.fillet * (1 - math.Pi/4)
			minimum := base - tc.radius*spandrel
			maximum := base - math.Sqrt(tc.radius*tc.radius-tc.fillet*tc.fillet)*spandrel
			require.GreaterOrEqual(t, volume.Value.Mag()+volume.Bound.Mag(), minimum)
			require.LessOrEqual(t, volume.Value.Mag()-volume.Bound.Mag(), maximum)
		})
	}
}

func TestRevolveQuarterCapRadialRimFilletRefusesOtherSources(t *testing.T) {
	for _, tc := range []struct {
		name                            string
		xlo, xhi, radius, angle, fillet float64
		selectStart                     bool
	}{
		{name: "non quarter turn", xhi: 10, radius: 8, angle: 45, fillet: 1},
		{name: "other axial cap", xhi: 10, radius: 8, angle: 90, fillet: 1, selectStart: true},
		{name: "outside bounded radius", xhi: 10, radius: 8, angle: 90, fillet: 4.5},
	} {
		t.Run(tc.name, func(t *testing.T) {
			doc, source := rimQuarterCylinder(t, tc.xlo, tc.xhi, tc.radius, tc.angle)
			x := tc.xhi
			if tc.selectStart {
				x = tc.xlo
			}
			_, err := decad.Fillet(t.Context(), source, quarterRim(x), units.Millimeters(tc.fillet))
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.Equal(t, []*decad.Body{source}, doc.Bodies())
		})
	}
}

// Independent angular quadrature checks the result's reported measurements
// against the source cylinder and the analytic quarter-cylinder trim. The
// production proof instead integrates the held mesh and encloses its missing
// occupied volume by station widths.
func TestRevolveQuarterCapRadialRimFilletMeasurements(t *testing.T) {
	const length, radius, fillet = 10.0, 8.0, 1.0
	_, source := rimQuarterCylinder(t, 0, length, radius, 90)
	out, err := decad.Fillet(t.Context(), source, quarterRim(length), units.Millimeters(fillet))
	require.NoError(t, err)
	volume, err := out.Volume()
	require.NoError(t, err)
	area, err := out.Area()
	require.NoError(t, err)
	centroid, err := out.Centroid()
	require.NoError(t, err)

	const samples = 8192
	simpson := func(hi float64, fn func(float64) float64) float64 {
		step := hi / samples
		sum := fn(0) + fn(hi)
		for i := 1; i < samples; i++ {
			weight := 2.0
			if i%2 != 0 {
				weight = 4
			}
			sum += weight * fn(float64(i)*step)
		}
		return sum * step / 3
	}
	width := func(z float64) float64 {
		if z >= fillet {
			return length
		}
		return length - fillet + math.Sqrt(2*fillet*z-z*z)
	}
	angleIntegral := func(fn func(z, h, jac float64) float64) float64 {
		return simpson(math.Pi/2, func(angle float64) float64 {
			z, h := radius*math.Sin(angle), radius*math.Cos(angle)
			return fn(z, h, radius*math.Cos(angle))
		})
	}
	wantVolume := angleIntegral(func(z, h, jac float64) float64 { return width(z) * h * jac })
	momentX := angleIntegral(func(z, h, jac float64) float64 {
		return width(z) * width(z) * h * jac / 2
	})
	momentY := angleIntegral(func(z, h, jac float64) float64 {
		return width(z) * h * h * jac / 2
	})
	momentZ := angleIntegral(func(z, h, jac float64) float64 { return z * width(z) * h * jac })
	capRemoved := simpson(math.Asin(fillet/radius), func(angle float64) float64 {
		h := radius * math.Cos(angle)
		return radius * h * math.Cos(angle)
	})
	angularEnd := angleIntegral(func(z, _, jac float64) float64 { return width(z) * jac })
	outerWall := simpson(math.Pi/2, func(angle float64) float64 {
		return radius * width(radius*math.Sin(angle))
	})
	blendWall := simpson(math.Pi/2, func(angle float64) float64 {
		z := fillet * (1 - math.Cos(angle))
		return fillet * math.Sqrt(radius*radius-z*z)
	})
	wantArea := math.Pi*radius*radius/2 - capRemoved + angularEnd +
		(length-fillet)*radius + outerWall + blendWall
	wantCentroid := r3.NewVec(momentX/wantVolume, momentY/wantVolume, momentZ/wantVolume)
	centroidGap := centroid.Value.Sub(wantCentroid).Len()

	require.InDelta(t, wantVolume, volume.Value.Mag(), 0.01)
	require.LessOrEqual(t, math.Abs(wantVolume-volume.Value.Mag()), volume.Bound.Mag()+1e-4)
	require.InDelta(t, wantArea, area.Value.Mag(), 0.1)
	require.LessOrEqual(t, math.Abs(wantArea-area.Value.Mag()), area.Bound.Mag()+1e-4)
	require.Less(t, centroidGap, 0.01)
	require.LessOrEqual(t, centroidGap, centroid.Bound.Mag()+1e-4)
}
