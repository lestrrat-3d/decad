package revolveloftfaces

import (
	"github.com/lestrrat-3d/decad/internal/loftface"
)

// SimpleLoop records the certified cells and walk direction of one source.
type SimpleLoop struct {
	Cells []loftface.SimpleCell
	CCW   bool
}
