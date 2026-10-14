package decad

import (
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/r3"
)

// producerID is the root spelling of provenance.Producer: the document-local
// identity assigned to one successful model operation. It supports provenance
// while keeping the model's execution bookkeeping private.
type producerID = provenance.Producer

func zeroVec(v r3.Vec) bool { return v == (r3.Vec{}) }
