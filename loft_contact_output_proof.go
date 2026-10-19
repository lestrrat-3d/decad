package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/loftcontact"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// Both proof modes charge their held audits to the output budget.
func certifyLoftOutputBoundary(ctx context.Context, first, second *Body,
	operation *proofbound.WorkBudget) (loftcontact.OutputBoundary, bool, error) {
	return certifyLoftOutputBoundaryMode(ctx, first, second, operation, false, nil)
}

func certifyLoftOutputBoundaryForDirectShell(ctx context.Context, first, second *Body,
	operation *proofbound.WorkBudget) (loftcontact.OutputBoundary, bool, error) {
	var sourceCertified bool
	proof, admitted, err := certifyLoftOutputBoundaryMode(ctx, first, second,
		operation, true, &sourceCertified)
	if err == nil && !admitted && sourceCertified {
		return loftcontact.OutputBoundary{}, false, errLoftDirectProofRefused
	}
	return proof, admitted, err
}

// certifyLoftOutputBoundaryMode stays in the root because it reads the *Body
// payloads and runs the root source partition. loftcontact builds and audits
// the output shell from that partition.
func certifyLoftOutputBoundaryMode(ctx context.Context, first, second *Body,
	operation *proofbound.WorkBudget, directShell bool,
	sourceCertified *bool) (loftcontact.OutputBoundary, bool, error) {
	var no loftcontact.OutputBoundary
	arithmetic := &loftcontact.Arithmetic{OK: true}
	if err := ctx.Err(); err != nil {
		return no, false, err
	}
	if first == nil || second == nil || operation == nil {
		return no, false, nil
	}
	certifySource := certifyLoftSourcePartition
	if directShell {
		certifySource = certifyLoftSourcePartitionForDirectShell
	}
	source, ok, err := certifySource(ctx, first, second, operation, arithmetic)
	if err != nil {
		return no, false, err
	}
	if !arithmetic.OK {
		if directShell {
			return no, false, errLoftDirectLimit
		}
		return no, false, nil
	}
	if !ok {
		return no, false, nil
	}
	if sourceCertified != nil {
		*sourceCertified = true
	}
	pp, prismOK := first.n.Payload.(prismPayload)
	lp, loftOK := second.n.Payload.(loftPayload)
	if !prismOK {
		pp, prismOK = second.n.Payload.(prismPayload)
		lp, loftOK = first.n.Payload.(loftPayload)
	}
	if !prismOK || !loftOK {
		return no, false, nil
	}
	outputLimit := uint64(65_536)
	if directShell {
		outputLimit = loftDirectOutputLimit(ctx)
	}
	return loftcontact.CertifyOutputBoundary(ctx, operation, pp, lp, source,
		outputLimit, directShell, arithmetic)
}
