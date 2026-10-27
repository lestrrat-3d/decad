package orderedwork

import "context"

// Proofs runs proof(ctx', k) for k in [0, n) and returns the error of the
// lowest failing k, the error a serial loop returns. With workers < 2 it is
// that serial loop, with the caller's ctx. Otherwise proofs run on worker
// goroutines, each call with a private context derived from context.WithoutCancel(ctx),
// so it keeps ctx's values but never sees ctx itself.
//
// Only the calling goroutine cancels a private context. It polls ctx.Err()
// every eachPollInterval and, when that fails, cancels every private context,
// waits for the proofs and returns ctx's own error. When proof k fails, the
// private contexts of every k' > k are cancelled: their results can no longer
// be the first failure. Every goroutine Proofs starts has finished when it
// returns. Proofs does not recover panics.
func Proofs(ctx context.Context, n, workers int, proof func(ctx context.Context, k int) error) error {
	if workers < 2 || n < 2 {
		for k := range n {
			if err := proof(ctx, k); err != nil {
				return err
			}
		}
		return nil
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	base := context.WithoutCancel(ctx)
	privates := make([]context.Context, n)
	cancels := make([]context.CancelFunc, n)
	for k := range n {
		// Each context derives from base, not from the previous one.
		privates[k], cancels[k] = context.WithCancel(base) //nolint:fatcontext
		defer cancels[k]()
	}
	cancelFrom := func(from int) {
		for _, cancel := range cancels[from:] {
			cancel()
		}
	}
	_, err := Each(n, workers, 1, func() error {
		if err := ctx.Err(); err != nil {
			cancelFrom(0)
			return err
		}
		return nil
	}, func(k int) error { //nolint:contextcheck // each proof gets its private context
		err := proof(privates[k], k)
		if err != nil {
			cancelFrom(k + 1)
		}
		return err
	})
	return err
}
