package decad

import (
	"github.com/lestrrat-3d/decad/internal/payloadclass"
)

// init installs the reader decadtest.PayloadClass calls. Tests need it because
// a Boolean result that keeps its operands' surfaces and curves
// (agent-docs/faceted-source-retention-design.md) no longer reveals the mesh path
// through any public reading.
func init() {
	payloadclass.Reader = func(body any) string {
		b, ok := body.(*Body)
		if !ok || b == nil {
			return ""
		}
		return payloadclass.TypeName(b.n.Payload)
	}
}
