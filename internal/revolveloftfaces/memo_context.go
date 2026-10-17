package revolveloftfaces

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/loftface"
)

func RevolveLoftFacesWithMeridianMemo(ctx context.Context) context.Context {
	return context.WithValue(ctx, RevolveLoftFacesMeridianMemoKey{}, loftface.NewMeridianMemo())
}
