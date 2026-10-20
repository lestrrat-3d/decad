package sweeprun

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/contactbox"
	"github.com/lestrrat-3d/decad/internal/contactround"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/seal"
	"github.com/lestrrat-3d/decad/internal/sweeppath"
	"github.com/lestrrat-3d/decad/internal/sweepproof"
	"github.com/lestrrat-3d/decad/internal/topo"
	vocab "github.com/lestrrat-3d/decad/report"
)

// Pair runs the sweep a validated body pair selects: the first producer whose
// source geometry admits both bodies and both paths certifies the sweep;
// otherwise the report is undecided. report names the pair and carries the
// caller's request.
func Pair(ctx context.Context, h Hooks, a, b *topo.Body, pa, pb affinePairPath, request vocab.SweepRequest,
	resolution *big.Rat, report *sweepReport) (*sweepReport, error) {
	if !pa.Supported || !pb.Supported {
		report.Outcome, report.Cause = vocab.SweepUndecided, vocab.SweepContactUnsupported
		report.Unresolved = &vocab.SweepInterval{From: sweeppath.Instant(new(big.Rat), pa.Duration),
			To: sweeppath.Instant(big.NewRat(1, 1), pa.Duration)}
		return report, nil
	}
	if pa.Drift != nil || pb.Drift != nil || pa.Screw != nil || pb.Screw != nil {
		if pa.Screw == nil && pb.Screw == nil {
			if result, admitted, err := sweepRotatingSpherePair(ctx, h, a, b, pa, pb, request, resolution, report); admitted {
				return result, err
			}
			if result, admitted, err := sweepRotatingSphereBox(ctx, h, a, b, pa, pb, request, resolution, report); admitted {
				return result, err
			}
		}
		return sweepRotatingPair(ctx, h, a, b, pa, pb, request, resolution, report)
	}
	boxA, okA := contactbox.SourceBoxAtPose(a, pa.From)
	boxB, okB := contactbox.SourceBoxAtPose(b, pb.From)
	if !okA || !okB {
		if okA {
			if _, faceted := b.Payload.(facetedrecord.Payload); faceted {
				return sweepFacetedFloor(ctx, h, a, b, pa, pb, request, report, boxA, false)
			}
		}
		if okB {
			if _, faceted := a.Payload.(facetedrecord.Payload); faceted {
				return sweepFacetedFloor(ctx, h, a, b, pa, pb, request, report, boxB, true)
			}
		}
		if !okA && !okB {
			sphereA, sphereOKA := contactround.SphereAtPose(a, pa.From)
			sphereB, sphereOKB := contactround.SphereAtPose(b, pb.From)
			if sphereOKA && sphereOKB {
				pair := [2]contactround.SphereProof{sphereA, sphereB}
				report.SetReplay(seal.Key{}, &sweepproof.Proof{PA: pa, PB: pb, SpherePair: &pair,
					Request: request.ContactRequest})
				return (&sourceSpherePairSweepRun{hooks: h, a: a, b: b, pa: pa, pb: pb,
					req: request, report: report, sphereA: sphereA, sphereB: sphereB}).execute(ctx, resolution)
			}
		}
		if okA && !okB {
			if sphere, ok := contactround.SphereAtPose(b, pb.From); ok {
				return (&sourceSphereSweepRun{hooks: h, a: a, b: b, pa: pa, pb: pb,
					req: request, report: report, sphere: sphere, box: boxA}).execute(ctx, resolution)
			}
			if cylinder, ok := contactround.CylinderAtPose(b, pb.From); ok {
				return sourceCylinderFaceSweep(ctx, h, a, b, pa, pb, request, report, cylinder, boxA, false)
			}
		}
		if okB && !okA {
			if sphere, ok := contactround.SphereAtPose(a, pa.From); ok {
				return (&sourceSphereSweepRun{hooks: h, a: a, b: b, pa: pa, pb: pb,
					req: request, report: report, sphere: sphere, box: boxB, sphereFirst: true}).execute(ctx, resolution)
			}
			if cylinder, ok := contactround.CylinderAtPose(a, pa.From); ok {
				return sourceCylinderFaceSweep(ctx, h, a, b, pa, pb, request, report, cylinder, boxB, true)
			}
		}
		if sphere, sphereOK := contactround.SphereAtPose(a, pa.From); sphereOK {
			if box, boxOK := contactbox.SourceOrientedBoxAtPose(b, pb.From); boxOK {
				return (&orientedSphereSweepRun{hooks: h, a: a, b: b, pa: pa, pb: pb,
					req: request, report: report, sphere: sphere, box: box, sphereFirst: true}).execute(ctx, resolution)
			}
		}
		if sphere, sphereOK := contactround.SphereAtPose(b, pb.From); sphereOK {
			if box, boxOK := contactbox.SourceOrientedBoxAtPose(a, pa.From); boxOK {
				return (&orientedSphereSweepRun{hooks: h, a: a, b: b, pa: pa, pb: pb,
					req: request, report: report, sphere: sphere, box: box}).execute(ctx, resolution)
			}
		}
		if _, orientedA := contactbox.SourceOrientedBoxAtPose(a, pa.From); orientedA {
			if _, orientedB := contactbox.SourceOrientedBoxAtPose(b, pb.From); orientedB {
				return sweepRotatingPair(ctx, h, a, b, pa, pb, request, resolution, report)
			}
		}
		if result, admitted, err := sweepPlanarPair(ctx, h, a, b, pa, pb, request, resolution, report); admitted {
			return result, err
		}
		report.Outcome, report.Cause = vocab.SweepUndecided, vocab.SweepContactUnsupported
		report.Unresolved = &vocab.SweepInterval{From: sweeppath.Instant(new(big.Rat), pa.Duration),
			To: sweeppath.Instant(big.NewRat(1, 1), pa.Duration)}
		return report, nil
	}
	report.SetReplay(seal.Key{}, &sweepproof.Proof{PA: pa, PB: pb, BoxA: boxA, BoxB: boxB,
		Request: request.ContactRequest})
	run := pairSweepRun{hooks: h, a: a, b: b, pa: pa, pb: pb, req: request, report: report,
		boxA: boxA, boxB: boxB}
	result, err := run.execute(ctx, resolution)
	if err != nil || result == nil {
		return result, err
	}
	result.Replay(seal.Key{}).Snapshot(result.Outcome)
	return result, nil
}
