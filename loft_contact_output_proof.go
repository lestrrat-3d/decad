package decad

import (
	"context"
	"errors"

	"github.com/lestrrat-3d/decad/internal/loftcontact"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// certifyLoftSourcePartition stays in the root because it reads the *Body
// payloads, in either operand order, and reads their source spans when a
// refused partition owes that read. It maps the direct-shell exact seam size
// refusal to errLoftDirectLimit. The partition proof is
// loftcontact.CertifySourcePartition.
func certifyLoftSourcePartition(ctx context.Context, first, second *Body,
	operation *proofbound.WorkBudget, directShell bool,
	checked ...*loftcontact.Arithmetic) (loftcontact.SourcePartition, bool, error) {
	var no loftcontact.SourcePartition
	if err := ctx.Err(); err != nil {
		return no, false, err
	}
	if first == nil || second == nil || operation == nil {
		return no, false, nil
	}
	prism, loft := first, second
	pp, prismOK := prism.n.Payload.(prismPayload)
	lp, loftOK := loft.n.Payload.(loftPayload)
	if !prismOK || !loftOK {
		prism, loft = second, first
		pp, prismOK = prism.n.Payload.(prismPayload)
		lp, loftOK = loft.n.Payload.(loftPayload)
		if !prismOK || !loftOK {
			return no, false, nil
		}
	}
	readSource := func(budget *proofbound.WorkBudget, withOrder bool) error {
		source, certified, err := certifyLoftSourceSpanCaps(ctx, budget, prism, loft)
		if err != nil || !certified || !withOrder {
			return err
		}
		_, _, err = certifyLoftCapSourceOrder(ctx, budget, source)
		return err
	}
	partition, admitted, err := loftcontact.CertifySourcePartition(ctx, operation,
		pp, lp, directShell, readSource, checked...)
	if errors.Is(err, loftcontact.ErrOrderedSeamSize) {
		return no, false, errLoftDirectLimit
	}
	return partition, admitted, err
}

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
// payloads and runs the root source partition wrapper. loftcontact builds and
// audits the output shell from that partition.
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
	source, ok, err := certifyLoftSourcePartition(ctx, first, second, operation, directShell, arithmetic)
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
