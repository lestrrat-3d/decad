package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestRevolveQuarterCapRadialRimChamferVerified(t *testing.T) {
	for _, tc := range []struct {
		name                      string
		xlo, xhi, radius, setback float64
	}{
		{name: "ten by eight", xlo: 0, xhi: 10, radius: 8, setback: 1},
		{name: "shifted six by five", xlo: -2, xhi: 4, radius: 5, setback: 0.5},
	} {
		t.Run(tc.name, func(t *testing.T) {
			doc, source := rimQuarterCylinder(t, tc.xlo, tc.xhi, tc.radius, 90)
			out, err := decad.Chamfer(t.Context(), source, quarterRim(tc.xhi), units.Millimeters(tc.setback))
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
					if origin.Role == "chamfer(0,1)" {
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
				r3.NewVec(tc.xhi-tc.setback, 0, 0),
				r3.NewVec(tc.xhi-tc.setback, tc.radius, 0),
				r3.NewVec(tc.xhi, 0, tc.setback),
				r3.NewVec(tc.xhi, math.Sqrt(tc.radius*tc.radius-tc.setback*tc.setback), tc.setback),
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
			triangle := tc.setback * tc.setback / 2
			minimum := base - tc.radius*triangle
			maximum := base - math.Sqrt(tc.radius*tc.radius-tc.setback*tc.setback)*triangle
			require.GreaterOrEqual(t, volume.Value.Mag()+volume.Bound.Mag(), minimum)
			require.LessOrEqual(t, volume.Value.Mag()-volume.Bound.Mag(), maximum)
		})
	}
}

func TestRevolveQuarterCapRadialRimChamferRefusesOtherSources(t *testing.T) {
	for _, tc := range []struct {
		name                             string
		xlo, xhi, radius, angle, setback float64
		selectStart                      bool
		asymmetric                       bool
	}{
		{name: "non quarter turn", xhi: 10, radius: 8, angle: 45, setback: 1},
		{name: "other axial cap", xhi: 10, radius: 8, angle: 90, setback: 1, selectStart: true},
		{name: "outside bounded setback", xhi: 10, radius: 8, angle: 90, setback: 4.5},
		{name: "asymmetric setback", xhi: 10, radius: 8, angle: 90, setback: 1, asymmetric: true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			doc, source := rimQuarterCylinder(t, tc.xlo, tc.xhi, tc.radius, tc.angle)
			x := tc.xhi
			if tc.selectStart {
				x = tc.xlo
			}
			var opts []decad.ChamferOption
			if tc.asymmetric {
				ref := decad.Faces(decad.Facing(r3.NewVec(1, 0, 0)))
				opts = append(opts, decad.WithAsymmetricChamfer(ref, units.Millimeters(0.5)))
			}
			_, err := decad.Chamfer(t.Context(), source, quarterRim(x), units.Millimeters(tc.setback), opts...)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.Equal(t, []*decad.Body{source}, doc.Bodies())
		})
	}
}

// Independent quadrature checks the reported mass properties against the
// planar chamfer and source cylinder. Production integrates the held mesh and
// encloses its missing occupied volume by certified station widths.
func TestRevolveQuarterCapRadialRimChamferMeasurements(t *testing.T) {
	const length, radius, setback = 10.0, 8.0, 1.0
	_, source := rimQuarterCylinder(t, 0, length, radius, 90)
	out, err := decad.Chamfer(t.Context(), source, quarterRim(length), units.Millimeters(setback))
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
		if z >= setback {
			return length
		}
		return length - setback + z
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
	capRemoved := simpson(math.Asin(setback/radius), func(angle float64) float64 {
		h := radius * math.Cos(angle)
		return radius * h * math.Cos(angle)
	})
	angularEnd := angleIntegral(func(z, _, jac float64) float64 { return width(z) * jac })
	outerWall := simpson(math.Pi/2, func(angle float64) float64 {
		return radius * width(radius*math.Sin(angle))
	})
	blendWall := math.Sqrt2 * simpson(setback, func(z float64) float64 {
		return math.Sqrt(radius*radius - z*z)
	})
	wantArea := math.Pi*radius*radius/2 - capRemoved + angularEnd +
		(length-setback)*radius + outerWall + blendWall
	wantCentroid := r3.NewVec(momentX/wantVolume, momentY/wantVolume, momentZ/wantVolume)
	centroidGap := centroid.Value.Sub(wantCentroid).Len()

	require.InDelta(t, wantVolume, volume.Value.Mag(), 0.01)
	require.LessOrEqual(t, math.Abs(wantVolume-volume.Value.Mag()), volume.Bound.Mag()+1e-4)
	require.InDelta(t, wantArea, area.Value.Mag(), 0.1)
	require.LessOrEqual(t, math.Abs(wantArea-area.Value.Mag()), area.Bound.Mag()+1e-4)
	require.Less(t, centroidGap, 0.01)
	require.LessOrEqual(t, centroidGap, centroid.Bound.Mag()+1e-4)
}
