package revolveloftfaces

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftface"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

func LfSourceBox(source *pointSectionRecord) ([4]*big.Rat, error) {
	var box [4]*big.Rat
	for i, v := range source.SourceBox {
		if proofbound.IsNonFinite(v) {
			return box,
				fmt.Errorf("%w: loft source rectangle is not finite", decaderr.ErrUnsupported)
		}
		box[i] = proofarith.FloatRat(v)
	}
	if box[0].Cmp(box[1]) >= 0 || box[2].Cmp(box[3]) >= 0 {
		return box, fmt.Errorf("%w: loft source rectangle has no area", decaderr.ErrUnsupported)
	}
	return box, nil
}

// Every true LoftFaces band is inside the image of the authenticated source
// rectangle. A circle missed by every full band image cannot meet the
// surviving Cut/Intersect cap. Exact coordinate hulls intentionally include
// more than the actual Sketch region and therefore only prove misses.
func ToolCircleOutsideSource(trim *facetedrecord.LoftFacesTrimRecord,
	x, radius *big.Rat) bool {
	if trim == nil || trim.Source == nil || x == nil || radius == nil ||
		radius.Sign() <= 0 {
		return false
	}
	box, err := LfSourceBox(trim.Source)
	if err != nil {
		return false
	}
	return loftface.CircleOutsideMaps(trim.Maps, box, x, radius)
}
