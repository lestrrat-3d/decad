package capband

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// BandRadius resolves a circular wall's offset radius and checks that the
// requested cap-level taper survives float64 at the wall's own scale. An
// empty offset is degenerate; a real taper that rounds back to the original
// radius is unsupported because its cone would be recorded as a cylinder.
func BandRadius(w survey2d.SideWalk, d, tol float64) (float64, error) {
	r, ok := offset2d.OffsetRadius(w, 1, d, tol)
	if !ok {
		return 0, offset2d.ErrDrop
	}
	if r == w.Radius {
		return 0, fmt.Errorf(`%w: the chamfer setback %v mm is below the float64 spacing of `+
			`a circular wall's own radius %v mm, so the cap contour's radial change rounds away `+
			`and the band's cone patch cannot be told from a cylinder; a wider setback or `+
			`a smaller radius states a chamfer this evaluator can build`, decaderr.ErrUnsupported, d, w.Radius)
	}
	return r, nil
}
