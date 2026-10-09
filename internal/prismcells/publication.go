package prismcells

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/sketch"
)

// CellProfiles reads arranged cells and restates split straight runs at the
// vertices that neighboring cells report on the same carrier. This gives a
// shared span the same edge key in every cell that walks it. An unresolved
// circular run declines the whole scene, returning no profiles for the caller
// to route to its mesh path.
func CellProfiles(ctx context.Context, budget *proofbound.WorkBudget, s *sketch.Sketch) ([]*sketch.Profile, error) {
	profiles, err := ProfilesContext(ctx, s.Profiles)
	if err != nil {
		return nil, err
	}
	split, resolved, err := SplitRuns(budget, profiles)
	if err != nil || !resolved {
		return nil, err
	}
	return split, nil
}

// ProfilesContext waits for sketch's synchronous arrangement publication even
// after cancellation, so the worker cannot outlive this operation.
func ProfilesContext(ctx context.Context, profiles func() []*sketch.Profile) ([]*sketch.Profile, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	done := make(chan []*sketch.Profile)
	go func() {
		done <- profiles()
	}()
	select {
	case result := <-done:
		return result, nil
	case <-ctx.Done():
		<-done
		return nil, ctx.Err()
	}
}

// ChainsContext gives the chain publication the same cancellation rule.
func ChainsContext(ctx context.Context, chains func() []*sketch.Chain) ([]*sketch.Chain, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	done := make(chan []*sketch.Chain)
	go func() { done <- chains() }()
	select {
	case result := <-done:
		return result, nil
	case <-ctx.Done():
		<-done
		return nil, ctx.Err()
	}
}
