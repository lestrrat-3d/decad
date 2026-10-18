// Package revolvetess assembles angular Revolve and chain Sweep meshes over
// topology nodes. Root owns public mesh handles and verification options.
package revolvetess

// Level mirrors the ordered public tessellation verification levels at this
// internal boundary.
type Level int

const (
	VerifyNone Level = iota
	VerifyBoundary
	VerifyAll
)
