package prismcells

import (
	"context"

	"github.com/lestrrat-3d/sketch"
)

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
