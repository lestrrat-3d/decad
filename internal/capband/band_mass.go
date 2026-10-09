package capband

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

// BandMassInput holds the root payload's recorded and built readings for one
// cap band. The root adapter authenticates the loop areas and contour before
// the volume and first-moment integrals read them.
type BandMassInput struct {
	Loop, CapBoundary sectionrecord.LoopRecord
	Patches           []Patch
	CapLevel          proofbound.BoundedScalar
	SideLevel         proofbound.BoundedScalar
	SideArea          proofbound.BoundedScalar
	CapArea           proofbound.BoundedScalar
	MaterialSign      float64
	Orientation       float64
	Delta             float64
	LevelDelta        float64
	Closure           Closure
}

// LevelVolume bounds the volume moved by the side level's displacement. The
// offset sections nest, so only columns through the larger section area can
// change where they leave the band toward the cap.
func LevelVolume(in BandMassInput) float64 {
	if in.LevelDelta <= 0 {
		return 0
	}
	areaMax := math.Max(proofbound.AbsSumUpper(in.SideArea.Value, in.SideArea.Bound),
		proofbound.AbsSumUpper(in.CapArea.Value, in.CapArea.Bound))
	return proofbound.ProductUpper(areaMax, in.LevelDelta)
}

// BandVolume integrates the band's two flat closing disks and its patches.
// Each term keeps its own bound before cancellation. The cap contour's
// displacement and the side level's displacement are charged once after the
// flux sum; closure slivers are charged before the divergence division.
func BandVolume(in BandMassInput, work *freeform.FreeformWork) (proofbound.BoundedScalar, error) {
	fluxTotal := proofbound.BoundedAdd(
		proofbound.BoundedMul(
			proofbound.MeasuredScalar(in.CapLevel.Value*(-in.MaterialSign), in.CapLevel.Bound), in.CapArea),
		proofbound.BoundedMul(
			proofbound.MeasuredScalar(in.SideLevel.Value*in.MaterialSign, in.CapLevel.Bound), in.SideArea),
	)
	patchAreaTotal := proofbound.BoundedScalar{}
	for _, g := range in.Patches {
		f := RawFlux(g)
		fluxTotal = proofbound.BoundedAdd(fluxTotal,
			proofbound.MeasuredScalar(-in.MaterialSign*in.Orientation*f.Value, f.Bound))
		pa, pb := AreaOf(g)
		patchAreaTotal = proofbound.BoundedAdd(patchAreaTotal, proofbound.MeasuredScalar(pa, pb))
	}
	if !in.Closure.Zero() {
		pointUpper, err := PointUpper(in.Loop, in.CapBoundary, in.Delta, in.Closure,
			in.SideLevel, in.CapLevel, work)
		if err != nil {
			return proofbound.BoundedScalar{}, err
		}
		fluxTotal.Bound = proofbound.AbsSumUpper(fluxTotal.Bound, in.Closure.FluxAllow(pointUpper,
			proofbound.AbsSumUpper(in.SideLevel.Value, in.SideLevel.Bound),
			proofbound.AbsSumUpper(in.CapLevel.Value, in.CapLevel.Bound)))
	}
	result := proofbound.BoundedQuotient(fluxTotal.Value, fluxTotal.Bound, 3, 0)
	areaUpper := proofbound.AbsSumUpper(patchAreaTotal.Value, patchAreaTotal.Bound,
		in.CapArea.Value, in.CapArea.Bound)
	result.Bound = proofbound.AbsSumUpper(result.Bound, proofbound.SweptVolumeAllow(in.Delta, areaUpper))
	result.Bound = proofbound.AbsSumUpper(result.Bound, LevelVolume(in))
	return result, nil
}

// BandMoment integrates the first moment of the same closed band as BandVolume.
// The two flat disks contribute only to the axial moment. Patch terms use
// the same orientation correction as the volume flux. The contour, locus,
// level and closure allowances are composed once per band.
func BandMoment(in BandMassInput, work *freeform.FreeformWork) (
	proofbound.BoundedScalar, proofbound.BoundedScalar, proofbound.BoundedScalar, error,
) {
	half := proofbound.ExactScalar(0.5)
	capZTerm := proofbound.BoundedMul(
		proofbound.BoundedMul(proofbound.BoundedMul(in.CapLevel, in.CapLevel), half),
		proofbound.BoundedMul(proofbound.ExactScalar(-in.MaterialSign), in.CapArea))
	sideZTerm := proofbound.BoundedMul(
		proofbound.BoundedMul(proofbound.BoundedMul(in.SideLevel, in.SideLevel), half),
		proofbound.BoundedMul(proofbound.ExactScalar(in.MaterialSign), in.SideArea))
	muTotal := proofbound.BoundedScalar{}
	mvTotal := proofbound.BoundedScalar{}
	mzTotal := proofbound.BoundedAdd(capZTerm, sideZTerm)

	patchAreaTotal := proofbound.BoundedScalar{}
	locusVolume, locusRadialGap := 0.0, 0.0
	for _, g := range in.Patches {
		pmu, pmv, pmz := FirstMomentFlux(g)
		sign := -in.MaterialSign * in.Orientation
		muTotal = proofbound.BoundedAdd(muTotal, proofbound.MeasuredScalar(sign*pmu.Value, pmu.Bound))
		mvTotal = proofbound.BoundedAdd(mvTotal, proofbound.MeasuredScalar(sign*pmv.Value, pmv.Bound))
		mzTotal = proofbound.BoundedAdd(mzTotal, proofbound.MeasuredScalar(sign*pmz.Value, pmz.Bound))
		pa, pb := AreaOf(g)
		patchAreaTotal = proofbound.BoundedAdd(patchAreaTotal, proofbound.MeasuredScalar(pa, pb))
		vol, gap := ChordLocusVolume(g)
		locusVolume = proofbound.AbsSumUpper(locusVolume, vol)
		locusRadialGap = math.Max(locusRadialGap, gap)
	}

	levelVolume := LevelVolume(in)
	if in.Delta > 0 || locusVolume > 0 || levelVolume > 0 {
		areaUpper := proofbound.AbsSumUpper(patchAreaTotal.Value, patchAreaTotal.Bound,
			in.CapArea.Value, in.CapArea.Bound)
		coordUpper, err := CoordUpper(in.Loop, in.CapBoundary, in.Delta,
			in.SideLevel, in.CapLevel, work)
		if err != nil {
			return proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, err
		}
		allow := proofbound.SweptMomentAllow(in.Delta, areaUpper, coordUpper)
		if locusVolume > 0 {
			locusReach := proofbound.AbsSumUpper(coordUpper, locusRadialGap)
			allow = proofbound.AbsSumUpper(allow, proofbound.ProductUpper(locusVolume, locusReach))
		}
		allow = proofbound.AbsSumUpper(allow, proofbound.ProductUpper(levelVolume, coordUpper))
		muTotal.Bound = proofbound.AbsSumUpper(muTotal.Bound, allow)
		mvTotal.Bound = proofbound.AbsSumUpper(mvTotal.Bound, allow)
		mzTotal.Bound = proofbound.AbsSumUpper(mzTotal.Bound, allow)
	}
	if !in.Closure.Zero() {
		pointUpper, err := PointUpper(in.Loop, in.CapBoundary, in.Delta, in.Closure,
			in.SideLevel, in.CapLevel, work)
		if err != nil {
			return proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, err
		}
		inPlane, axial := in.Closure.MomentAllow(pointUpper,
			proofbound.AbsSumUpper(in.SideLevel.Value, in.SideLevel.Bound),
			proofbound.AbsSumUpper(in.CapLevel.Value, in.CapLevel.Bound))
		muTotal.Bound = proofbound.AbsSumUpper(muTotal.Bound, inPlane)
		mvTotal.Bound = proofbound.AbsSumUpper(mvTotal.Bound, inPlane)
		mzTotal.Bound = proofbound.AbsSumUpper(mzTotal.Bound, axial)
	}
	return muTotal, mvTotal, mzTotal, nil
}

// CentroidGeometryBound bounds a centroid's distance from an estimate by the
// farthest of the box's eight corners, then adds the box's displacement. The
// distance is convex, so its maximum over the box is attained at a corner.
func CentroidGeometryBound(estimate r3.Vec, bounds measurement.Box) float64 {
	xs := [2]float64{bounds.Min.X, bounds.Max.X}
	ys := [2]float64{bounds.Min.Y, bounds.Max.Y}
	zs := [2]float64{bounds.Min.Z, bounds.Max.Z}
	reach := 0.0
	for _, x := range xs {
		for _, y := range ys {
			for _, z := range zs {
				dd := r3.NewVec(x, y, z).Sub(estimate).Len()
				if dd > reach {
					reach = dd
				}
			}
		}
	}
	return proofbound.AbsSumUpper(reach, bounds.Bound.Mag())
}
