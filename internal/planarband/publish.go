// Package planarband publishes the contact report of two admitted planar
// solids from the exact planar kernel's relation over their held meshes
// (docs/multibody-dynamics-design.md §9.3–§9.6, §10.4, §10.5). The root
// package admits the bodies, builds their snapshots at the query poses and
// caches their §9.2 certificates; this package turns the kernel's answer into
// the relation, gap, manifold and reason.
package planarband

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/pair"
	"github.com/lestrrat-3d/decad/internal/pair/planar"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/reportvocab"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/units"
)

// Convexity reads a body's §9.2 convexity certificate. The root package
// implements it: the certificate is cached on the body and built from its
// payload.
type Convexity interface {
	Convex(ctx context.Context, budget *proofbound.WorkBudget, body *topo.Body) (bool, error)
}

// Publish writes the report of result, the exact planar relation of a and b,
// which are the held solids of report.A and report.B at the query poses with
// deltaA and deltaB their displacements there, zero for an exact body. It
// first clears the relation, gap, overlap and manifold and sets the reason
// the kernel names. A pair with a positive displacement takes §10.4's band
// rule; an exact pair publishes the kernel's relation, with §10.5's support
// band on a separated pair and §9.3's manifold on a touch or an overlap.
// Publish does not retain certificates.
func Publish(ctx context.Context, budget *proofbound.WorkBudget, report *reportvocab.ContactReport,
	result planar.PlanarResult, a, b *planar.PlanarSolid, deltaA, deltaB proofarith.Dyadic,
	certificates Convexity) error {
	report.Relation, report.Gap, report.Overlap, report.Manifold = reportvocab.ContactUndecided, nil, nil, nil
	report.Reason = reportvocab.ContactReasonOf(result.Reason)
	if deltaA.Sign() > 0 || deltaB.Sign() > 0 {
		p := bandPair{a: a, b: b, deltaA: deltaA, deltaB: deltaB, certificates: certificates}
		return p.classify(ctx, budget, report, result)
	}
	band := SupportBand(report.Request)
	switch result.Relation {
	case pair.Separated:
		report.Relation = reportvocab.ContactSeparated
		gap := result.Gap.Measurement()
		report.Gap = &gap
		return publishSupportBand(budget, report, a, b, result, band)
	case pair.Touching, pair.Overlapping:
		report.Relation = reportvocab.ContactOverlapping
		if result.Relation == pair.Touching {
			report.Relation = reportvocab.ContactTouching
			gap := measurement.Measurement{Value: units.Millimeters(0), Exactness: measurement.Exact, Bound: units.Millimeters(0)}
			report.Gap = &gap
		}
		// §9.3 and §9.6 need one convex side for a manifold; without one an
		// overlap's reason names the missing certificate, and a touch takes
		// §10.5's non-convex guest path inside publishManifold.
		convexA, convexB, err := convexities(ctx, budget, report, certificates)
		if err != nil {
			return err
		}
		if !convexA && !convexB && result.Relation == pair.Overlapping {
			report.Reason = reportvocab.ContactNonConvex
			return nil
		}
		report.Reason = reportvocab.ContactNoNormalProof
		return publishManifold(budget, report, a, b, result, convexA, convexB, band)
	default:
		if report.Reason == reportvocab.ContactNoReason {
			report.Reason = reportvocab.ContactAmbiguousFeature
		}
		return nil
	}
}

// SupportBand is the request's SupportBand as an exact dyadic, zero for the
// zero Value.
func SupportBand(req reportvocab.ContactRequest) proofarith.Dyadic {
	band, ok := proofarith.DyOf(req.SupportBand.Base())
	if !ok || band.Sign() < 0 {
		return proofarith.DyZero()
	}
	return band
}

// convexities reads both bodies' certificates, A's first.
func convexities(ctx context.Context, budget *proofbound.WorkBudget, report *reportvocab.ContactReport,
	certificates Convexity) (bool, bool, error) {
	convexA, err := certificates.Convex(ctx, budget, report.A)
	if err != nil {
		return false, false, err
	}
	convexB, err := certificates.Convex(ctx, budget, report.B)
	return convexA, convexB, err
}
