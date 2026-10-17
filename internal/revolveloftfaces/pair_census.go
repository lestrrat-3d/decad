package revolveloftfaces

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/loftface"
)

type RevolveLoftFacesMeridianMemoKey struct{}

func RevolveLoftFacesMeridianMemoOf(ctx context.Context) *loftface.MeridianMemo {
	if ctx == nil {
		return nil
	}
	memo, _ := ctx.Value(RevolveLoftFacesMeridianMemoKey{}).(*loftface.MeridianMemo)
	return memo
}
