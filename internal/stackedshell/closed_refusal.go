package stackedshell

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// RefuseClosed is Shell's answer to WithNoOpenings on every receiver but a
// full-turn revolve over a hole-free meridian, a straight prism, a brep record
// that reads as a prism (inward along any axis, outward along its one axis),
// and an admitted two-slab rectangular boss.
// Their own builders run first (agent-docs/modify-reach-design.md Tables RX/SX,
// §14; agent-docs/modify-general-design.md §3.2). Each receiver gets the row
// that stages it: a faceted boolean result SX9, a brep or stacked receiver
// agent-docs/modify-general-design.md's SG2, a holed revolve meridian SX8, a
// partial revolve SX8 (it keeps both angular caps), and any other receiver
// base S3. Shell routes every straight prism to prismshell.BuildClosed, so
// none arrives here. Every one is ErrUnsupported: the closed body exists, and this
// evaluator does not build it.
func RefuseClosed(payload topo.Payload) error {
	switch p := payload.(type) {
	case facetedrecord.Payload:
		return fmt.Errorf(`%w: this evaluator modifies no faceted boolean result (modify-reach SX9)`, decaderr.ErrUnsupported)
	case breprecord.Payload, stackedrecord.Payload:
		return fmt.Errorf(`%w: this evaluator does not build a closed shell of this brep or stacked receiver; it builds one of a brep record that reads as a prism along a reference axis, or inward of a nested two-slab rectangular boss (modify-general SG2)`, decaderr.ErrUnsupported)
	case revolveplan.Payload:
		if len(p.Profile.Holes) > 0 {
			return fmt.Errorf(`%w: a closed shell of a revolve whose meridian holds a hole is outside the shell extension (modify-reach SX8)`, decaderr.ErrUnsupported)
		}
		// Shell routes a full turn over a hole-free meridian to shellRevolve,
		// so the revolve that arrives here is a partial turn.
		return fmt.Errorf(`%w: a closed shell of a partial revolve keeps both angular caps, whose walls are planes at constant distance that no revolve holds (modify-reach SX8)`, decaderr.ErrUnsupported)
	default:
		return fmt.Errorf(`%w: this evaluator shells a straight prism only`, decaderr.ErrUnsupported)
	}
}
