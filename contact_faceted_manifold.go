package decad

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/pair/planar"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/reportvocab"

	"github.com/lestrrat-3d/decad/internal/pair"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/units"
)

// This file admits a proven exact planar relation to a contact manifold
// (docs/multibody-dynamics-design.md §9.3) and maps the kernel's face ids
// back to the bodies' live topology. internal/pair builds the exact points;
// contact_faceted_patch.go turns them into bounded witnesses.

// publishPlanarManifold computes the manifold of a Touching pair, or the
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
func publishPlanarManifold(budget *proofbound.WorkBudget, report *reportvocab.ContactReport, a, b *planar.PlanarSolid,
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
				report.Reason = reportvocab.ContactNonConvex
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
			report.Reason = sourceBoxReason(manifold.Reason)
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
				report.Reason = sourceBoxReason(reason)
			}
			return nil
		}
		overlap = true
	default:
		return nil
	}
	features, err := reportvocab.NewPlanarFeatureMap(
		bodyNode(bodyHandle(report.A)), bodyNode(bodyHandle(report.B)), a, b)
	if err != nil {
		return err
	}
	manifold, reason := planarPatchManifold(report.Request, points, features)
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
			extra, reason := planarPatchManifold(report.Request, lifted, features)
			if extra == nil {
				report.Reason = reason
				return nil
			}
			manifold.Points = append(manifold.Points, extra.Points...)
		}
	}
	report.Manifold, report.Reason = manifold, reportvocab.ContactNoReason
	return nil
}

// planarSupportBand publishes an exact separated pair apart by at most the
// request's SupportBand as ContactBand (§10.5): its gap's upper end must be
// within the band and some support plane must hold a nonempty lifted set.
// The planes are read with the B body's faces first, then A's, each in face
// order. The gap becomes [0 ± g], g its upper end, and the manifold is every
// such plane's lifted set. A pair that meets neither stays Separated.
func planarSupportBand(budget *proofbound.WorkBudget, report *reportvocab.ContactReport, a, b *planar.PlanarSolid,
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
	report.Relation, report.Reason = reportvocab.ContactBand, reportvocab.ContactNoReason
	report.Gap = &measurement.Measurement{Value: units.Millimeters(0), Bound: units.Millimeters(g), Exactness: measurement.ExactnessOf(g)}
	features, err := reportvocab.NewPlanarFeatureMap(
		bodyNode(bodyHandle(report.A)), bodyNode(bodyHandle(report.B)), a, b)
	if err != nil {
		return err
	}
	manifold, reason := planarPatchManifold(report.Request, lifted, features)
	report.Manifold, report.Reason = manifold, reason
	return nil
}

// supportBandOf is the request's SupportBand as an exact dyadic, zero for the
// zero Value.
func supportBandOf(req reportvocab.ContactRequest) proofarith.Dyadic {
	band, ok := proofarith.DyOf(req.SupportBand.Base())
	if !ok || band.Sign() < 0 {
		return proofarith.DyZero()
	}
	return band
}
