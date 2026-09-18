package decad

import (
	"fmt"

	"github.com/lestrrat-3d/r3"
)

// producerID is the document-local identity assigned to one successful model
// operation. It supports provenance while keeping the model's execution
// bookkeeping private.
type producerID int

// operationKind is the evaluator's private operation vocabulary.
type operationKind int

const (
	opUnion operationKind = iota
	opCut
	opIntersect
)

func (k operationKind) String() string {
	switch k {
	case opUnion:
		return "union"
	case opCut:
		return "cut"
	case opIntersect:
		return "intersect"
	default:
		return fmt.Sprintf("operationKind(%d)", int(k))
	}
}

func zeroVec(v r3.Vec) bool { return v == (r3.Vec{}) }
