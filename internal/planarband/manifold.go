package planarband

import (
	"math/big"
	"slices"
	"sort"

	"github.com/lestrrat-3d/decad/internal/planarfeaturemap"
	vocab "github.com/lestrrat-3d/decad/report"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/pair"
	pairbox "github.com/lestrrat-3d/decad/internal/pair/box"
	"github.com/lestrrat-3d/decad/internal/pair/planar"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/units"
)

// publishManifold computes the manifold of a Touching pair, or the
// shallow-penetration patch of an Overlapping pair, and publishes it on
// report. An Overlapping pair takes planar.OverlapPatch. A Touching pair with
// no convex body publishes the contact set of a host face that holds every
// contact (planar.PlanarGuestTouch, §10.5). A withheld manifold leaves
// report.Manifold nil with its reason: AmbiguousFeature for a contact set
// outside §9.3's table or an overlap through two faces, NonConvex for a touch
// of two non-convex bodies that no host face covers, PointTooCoarse or
// NoNormalProof for a witness or normal over the request. A penetration that
// neither patch can certify otherwise keeps the reason report already carries.
// A positive band appends the lifted set of each support plane after the
// exact points (docs/multibody-dynamics-design.md §10.5).
func publishManifold(budget *proofbound.WorkBudget, report *contactReport, a, b *planar.PlanarSolid,
	result planar.PlanarResult, convexA, convexB bool, band proofarith.Dyadic) error {
	var points []planar.PatchPoint
	var planes []planar.SupportPlane
	overlap := false
	switch result.Relation {
	case pair.Touching:
		if !convexA && !convexB {
			// §10.5: a non-convex guest whose contacts all lie on one host face.
			manifold, err := planar.PlanarGuestTouch(a, b, result.Contacts, budget.Step)
			if err != nil {
				return err
			}
			if manifold.Points == nil {
				report.Reason = vocab.ContactNonConvex
				return nil
			}
			points, planes = manifold.Points, manifold.Supports
			break
		}
		manifold, err := planar.PlanarTouchManifold(a, b, result.Contacts, convexA, convexB, budget.Step)
		if err != nil {
			return err
		}
		if manifold.Points == nil {
			report.Reason = pair.ContactReasonOf(manifold.Reason)
			return nil
		}
		points, planes = manifold.Points, manifold.Supports
	case pair.Overlapping:
		var reason pair.Reason
		var err error
		points, planes, reason, err = planar.OverlapPatch(a, b, result, convexA, convexB,
			proofarith.DyZero(), proofarith.DyZero(), budget.Step)
		if err != nil {
			return err
		}
		if points == nil {
			if reason != pair.NoReason {
				report.Reason = pair.ContactReasonOf(reason)
			}
			return nil
		}
		overlap = true
	default:
		return nil
	}
	features, err := planarfeaturemap.NewPlanarFeatureMap(report.A, report.B, a, b)
	if err != nil {
		return err
	}
	manifold, reason := patchManifold(report.Request, points, features)
	if manifold == nil {
		report.Reason = reason
		return nil
	}
	if band.Sign() > 0 {
		lifted, err := planar.PlanarSupportSets(a, b, result, planes, band, overlap, budget.Step)
		if err != nil {
			return err
		}
		if lifted != nil {
			extra, reason := patchManifold(report.Request, lifted, features)
			if extra == nil {
				report.Reason = reason
				return nil
			}
			manifold.Points = append(manifold.Points, extra.Points...)
		}
	}
	report.Manifold, report.Reason = manifold, vocab.ContactNoReason
	return nil
}

// publishSupportBand publishes an exact separated pair apart by at most the
// request's SupportBand as ContactBand (§10.5): its gap's upper end must be
// within the band and some support plane must hold a nonempty lifted set.
// The planes are read with the B body's faces first, then A's, each in face
// order. The gap becomes [0 ± g], g its upper end, and the manifold is every
// such plane's lifted set. A pair that meets neither stays Separated.
func publishSupportBand(budget *proofbound.WorkBudget, report *contactReport, a, b *planar.PlanarSolid,
	result planar.PlanarResult, band proofarith.Dyadic) error {
	if band.Sign() <= 0 || result.Gap == nil {
		return nil
	}
	upper := new(big.Rat).Add(proofarith.FloatRat(result.Gap.ValueMM), proofarith.FloatRat(result.Gap.BoundMM))
	if upper.Cmp(band.Rat()) > 0 {
		return nil
	}
	lifted, err := planar.PlanarSupportSets(a, b, result, planar.SupportPlanes(a, b, true, true),
		band, false, budget.Step)
	if err != nil || len(lifted) == 0 {
		return err
	}
	g := proofbound.RatFloatUp(upper)
	if !measurement.FiniteValues(g) {
		return nil
	}
	report.Relation, report.Reason = vocab.ContactBand, vocab.ContactNoReason
	report.Gap = &measurement.Measurement{Value: units.Millimeters(0), Bound: units.Millimeters(g), Exactness: measurement.ExactnessOf(g)}
	features, err := planarfeaturemap.NewPlanarFeatureMap(report.A, report.B, a, b)
	if err != nil {
		return err
	}
	manifold, reason := patchManifold(report.Request, lifted, features)
	report.Manifold, report.Reason = manifold, reason
	return nil
}

// The points of a planar manifold publish as §9.4 states: each exact witness
// is converted once to a float with an outward ball, exactly as the shipped
// clips do (pairbox.PointMeasurement); each exact normal direction is
// normalized with an enclosing ball (pairbox.PlanarNormalMeasurement);
// entries are ordered by A's feature, then B's, then exact coordinate (§9.5).

type patchEntry struct {
	point      contactPoint
	keyA, keyB [2]int
	onA, onB   planar.Point3
}

// patchManifold converts the kernel's exact points to a contact manifold, or
// reports why it is withheld: a feature without one source identity
// (AmbiguousFeature), a witness ball over PointResolution (PointTooCoarse), or
// a normal angle over NormalResolution (NoNormalProof). A point the kernel
// gives no Separation publishes an exact zero.
func patchManifold(req vocab.ContactRequest, points []planar.PatchPoint,
	features *planarfeaturemap.PlanarFeatureMap) (*contactManifold, vocab.ContactReason) {
	entries := make([]patchEntry, 0, len(points))
	for _, p := range points {
		featureA, okA := features.Feature(0, p.A)
		featureB, okB := features.Feature(1, p.B)
		if !okA || !okB {
			return nil, vocab.ContactAmbiguousFeature
		}
		onA, validA := pairbox.PointMeasurement([3]*big.Rat(p.OnA))
		onB, validB := pairbox.PointMeasurement([3]*big.Rat(p.OnB))
		if !validA || !validB || onA.Bound.Base() > req.PointResolution.Base() ||
			onB.Bound.Base() > req.PointResolution.Base() {
			return nil, vocab.ContactPointTooCoarse
		}
		normal, angle, ok := pairbox.PlanarNormalMeasurement(p.Normal)
		if !ok || angle.Base() > req.NormalResolution.Base() {
			return nil, vocab.ContactNoNormalProof
		}
		point := contactPoint{OnA: onA, OnB: onB, Normal: normal, NormalAngle: angle,
			Separation: p.Separation.Measurement(), FaceA: featureA.Face, FaceB: featureB.Face,
			FeatureA: featureA, FeatureB: featureB}
		entries = append(entries, patchEntry{point: point, keyA: features.Order(0, featureA),
			keyB: features.Order(1, featureB), onA: p.OnA, onB: p.OnB})
	}
	sort.SliceStable(entries, func(i, j int) bool { return patchEntryLess(&entries[i], &entries[j]) })
	out := make([]contactPoint, 0, len(entries))
	for i := range entries {
		if i > 0 && !patchEntryLess(&entries[i-1], &entries[i]) {
			// Equal keys and equal exact coordinates: the same entry reached
			// through two pieces, merged under certified equality.
			continue
		}
		out = append(out, entries[i].point)
	}
	return &contactManifold{Points: out}, vocab.ContactNoReason
}

func patchEntryLess(x, y *patchEntry) bool {
	for _, c := range []int{slices.Compare(x.keyA[:], y.keyA[:]), slices.Compare(x.keyB[:], y.keyB[:]),
		comparePoint3(x.onA, y.onA), comparePoint3(x.onB, y.onB)} {
		if c != 0 {
			return c < 0
		}
	}
	return false
}

func comparePoint3(x, y planar.Point3) int {
	for axis := range 3 {
		if c := x[axis].Cmp(y[axis]); c != 0 {
			return c
		}
	}
	return 0
}
