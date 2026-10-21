package contactpair

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/boundaryaudit"
	vocab "github.com/lestrrat-3d/decad/report"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"

	"github.com/lestrrat-3d/decad/internal/clearance/pairkernel"
	"github.com/lestrrat-3d/decad/internal/contactbox"
	"github.com/lestrrat-3d/decad/internal/contactfaceted"
	"github.com/lestrrat-3d/decad/internal/contactround"
	"github.com/lestrrat-3d/decad/internal/contactruling"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/planarband"
	"github.com/lestrrat-3d/decad/internal/planarpose"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/verifiedmesh"
)

// Classify is ContactPair's proof for validated, distinct, live
// solids and a validated request, before the memo.
func Classify(ctx context.Context, mesher verifiedmesh.Mesher, a, b *topo.Body, poseA, poseB r3.Transform,
	req vocab.ContactRequest) (*contactReport, error) {
	report := &contactReport{A: a, B: b, PoseA: poseA, PoseB: poseB, Request: req}
	boxA, okA := contactbox.SourceBoxAtPose(a, poseA)
	boxB, okB := contactbox.SourceBoxAtPose(b, poseB)
	if !okA && !okB {
		sphereA, sphereOKA := contactround.SphereAtPose(a, poseA)
		sphereB, sphereOKB := contactround.SphereAtPose(b, poseB)
		if sphereOKA && sphereOKB {
			contactround.ClassifySpherePair(report, sphereA, sphereB)
			return report, nil
		}
	}
	if okA && !okB {
		if _, faceted := b.Payload.(facetedrecord.Payload); faceted {
			if err := contactfaceted.ClassifyFloorBox(ctx, report, b, poseB, boxA, false); err != nil {
				return nil, err
			}
			if err := classifyPlanarFallback(ctx, mesher, report); err != nil {
				return nil, err
			}
			return report, nil
		}
		if sphere, ok := contactround.SphereAtPose(b, poseB); ok {
			contactround.ClassifySphereBox(report, sphere, boxA, false)
			return report, nil
		}
		if cylinder, ok := contactround.CylinderAtPose(b, poseB); ok {
			contactround.ClassifyCylinderBox(report, cylinder, boxA, false)
			if err := classifyUndecidedCylinder(ctx, mesher, report); err != nil {
				return nil, err
			}
			return report, nil
		}
	}
	if okB && !okA {
		if _, faceted := a.Payload.(facetedrecord.Payload); faceted {
			if err := contactfaceted.ClassifyFloorBox(ctx, report, a, poseA, boxB, true); err != nil {
				return nil, err
			}
			if err := classifyPlanarFallback(ctx, mesher, report); err != nil {
				return nil, err
			}
			return report, nil
		}
		if sphere, ok := contactround.SphereAtPose(a, poseA); ok {
			contactround.ClassifySphereBox(report, sphere, boxB, true)
			return report, nil
		}
		if cylinder, ok := contactround.CylinderAtPose(a, poseA); ok {
			contactround.ClassifyCylinderBox(report, cylinder, boxB, true)
			if err := classifyUndecidedCylinder(ctx, mesher, report); err != nil {
				return nil, err
			}
			return report, nil
		}
	}
	if !okA || !okB {
		orientedA, orientedOKA := contactbox.SourceOrientedBoxAtPose(a, poseA)
		orientedB, orientedOKB := contactbox.SourceOrientedBoxAtPose(b, poseB)
		if orientedOKA && !orientedOKB {
			if sphere, ok := contactround.SphereAtPose(b, poseB); ok {
				contactround.ClassifySphereOrientedBox(report, sphere, orientedA, false)
				return report, nil
			}
		}
		if orientedOKB && !orientedOKA {
			if sphere, ok := contactround.SphereAtPose(a, poseA); ok {
				contactround.ClassifySphereOrientedBox(report, sphere, orientedB, true)
				return report, nil
			}
		}
		if orientedOKA && orientedOKB {
			contactbox.ClassifyOriented(report, orientedA, orientedB)
			if err := classifyPlanarManifold(ctx, mesher, report); err != nil {
				return nil, err
			}
			if err := classifyPlanarSupportBand(ctx, mesher, report); err != nil {
				return nil, err
			}
			return report, nil
		}
		planar, err := planarpose.ClassifyExactPair(ctx, mesher, report)
		if err != nil {
			return nil, err
		}
		if planar && report.Relation != vocab.ContactUndecided {
			return report, nil
		}
		identity := poseA == r3.Identity() && poseB == r3.Identity()
		if !planar && !identity {
			placed, err := contactruling.ClassifyPlacedRuling(ctx, mesher, report)
			if err != nil {
				return nil, err
			}
			if placed {
				return report, nil
			}
		}
		if identity {
			report.Reason = vocab.ContactNoReason
			if err := classifyAnalyticContact(ctx, report); err != nil {
				return nil, err
			}
			return report, nil
		}
		if !planar {
			report.Reason = vocab.ContactPayloadUnsupported
		}
		return report, nil
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	contactbox.ClassifySourceBoxes(report, boxA, boxB)
	return report, nil
}

// classifyPlanarFallback replaces an undecided source-box/faceted report with
// the exact planar relation when both bodies are admitted and it decides one.
func classifyPlanarFallback(ctx context.Context, mesher verifiedmesh.Mesher, report *contactReport) error {
	if report.Relation != vocab.ContactUndecided {
		return nil
	}
	trial := contactReport{A: report.A, B: report.B, PoseA: report.PoseA, PoseB: report.PoseB,
		Request: report.Request}
	planar, err := planarpose.ClassifyExactPair(ctx, mesher, &trial)
	if err != nil {
		return err
	}
	if planar && trial.Relation != vocab.ContactUndecided {
		*report = trial
	}
	return nil
}

// classifyPlanarManifold completes an oriented source-box report whose box
// patches publish no manifold for a touching or overlapping pair: both boxes
// are exact planar bodies (agent-docs/multibody-dynamics-design.md §9), so §9.3's
// manifold replaces the report when the exact planar relation agrees and
// every published point is an edge or vertex of one box on, in or crossing
// the other, the rows the box patches do not cover. Face pairs stay with the
// box patches (§9.4), and a report the planar path cannot complete stands.
func classifyPlanarManifold(ctx context.Context, mesher verifiedmesh.Mesher, report *contactReport) error {
	if report.Manifold != nil || report.Reason == vocab.ContactPointTooCoarse ||
		report.Relation != vocab.ContactTouching && report.Relation != vocab.ContactOverlapping {
		return nil
	}
	trial := contactReport{A: report.A, B: report.B, PoseA: report.PoseA, PoseB: report.PoseB,
		Request: report.Request}
	planar, err := planarpose.ClassifyExactPair(ctx, mesher, &trial)
	if err != nil {
		return err
	}
	if !planar || trial.Relation != report.Relation || trial.Manifold == nil {
		return nil
	}
	for _, point := range trial.Manifold.Points {
		if point.FeatureA.Face != nil && point.FeatureB.Face != nil {
			return nil
		}
	}
	*report = trial
	return nil
}

// classifyUndecidedCylinder completes a source cylinder report the box
// proofs left undecided: at identity query poses the analytic kernel's
// verdict, elsewhere the placed ruling of contactruling.ClassifyPlacedRuling.
func classifyUndecidedCylinder(ctx context.Context, mesher verifiedmesh.Mesher, report *contactReport) error {
	if report.Relation != vocab.ContactUndecided {
		return nil
	}
	if report.PoseA == r3.Identity() && report.PoseB == r3.Identity() {
		report.Reason = vocab.ContactNoReason
		return classifyAnalyticContact(ctx, report)
	}
	_, err := contactruling.ClassifyPlacedRuling(ctx, mesher, report)
	return err
}

// classifyPlanarSupportBand completes an oriented source-box report that may
// be separated by at most the request's positive SupportBand: the box patches
// know no band, so the exact planar path decides whether the pair is §10.5's
// ContactBand, and its report replaces this one when it is.
func classifyPlanarSupportBand(ctx context.Context, mesher verifiedmesh.Mesher, report *contactReport) error {
	if planarband.SupportBand(report.Request).Sign() <= 0 || report.Relation != vocab.ContactSeparated || report.Gap == nil ||
		report.Gap.Value.Base()-report.Gap.Bound.Base() > report.Request.SupportBand.Base() {
		return nil
	}
	trial := contactReport{A: report.A, B: report.B, PoseA: report.PoseA, PoseB: report.PoseB,
		Request: report.Request}
	planar, err := planarpose.ClassifyExactPair(ctx, mesher, &trial)
	if err != nil {
		return err
	}
	if planar && trial.Relation == vocab.ContactBand {
		*report = trial
	}
	return nil
}

// classifyAnalyticContact consumes the clearance kernel's complete-pair
// verdict. Only a touch the kernel proved by a ruling certificate keeps its
// faces and feet, so only that touch can publish a manifold here.
func classifyAnalyticContact(ctx context.Context, report *contactReport) error {
	res, err := pairkernel.Pair(ctx, report.A, report.B, false, nil)
	if err != nil {
		return err
	}
	switch res.Verdict {
	case boundaryaudit.PairDisjoint:
		gap := pairkernel.GapMeasurement(res)
		if !measurement.FiniteValues(gap.Value.Base(), gap.Bound.Base()) ||
			gap.Value.Base()-gap.Bound.Base() <= 0 {
			report.Reason = vocab.ContactNoGapProof
			return nil
		}
		report.Relation = vocab.ContactSeparated
		report.Gap = &gap
	case boundaryaudit.PairTouching:
		report.Relation = vocab.ContactTouching
		gap := measurement.Measurement{Value: units.Millimeters(0), Exactness: measurement.Exact, Bound: units.Millimeters(0)}
		report.Gap = &gap
		report.Reason = vocab.ContactNoNormalProof
		if res.Ruling != nil {
			contactruling.PublishRulingManifold(report, res.Ruling)
		}
	case boundaryaudit.PairOverlapping:
		report.Relation = vocab.ContactOverlapping
		report.Reason = vocab.ContactNoNormalProof
	default:
		report.Reason = vocab.ContactNoGapProof
	}
	return nil
}
