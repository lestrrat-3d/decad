package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
)

// The current point source and profile admission run before the moved
// whole-cell walk. A stale source cannot reach a simple-loop certificate.
func revolveLoftFacesSimpleSourceLoop(ctx context.Context,
	trim *loftFacesTrimRecord) (*revolveloftfaces.SimpleLoop, bool, error) {
	if ctx == nil || trim == nil || trim.Source == nil ||
		!pointSourceCurrent(trim.Source) ||
		trim.Source.Source.Profile == nil ||
		!trim.Source.Source.Profile.Valid ||
		len(trim.Source.Source.Profile.Holes) != 0 {
		return nil, false, nil
	}
	return revolveloftfaces.SimpleSourceLoop(ctx, trim)
}
