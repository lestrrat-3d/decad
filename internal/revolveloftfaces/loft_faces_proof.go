package revolveloftfaces

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
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
