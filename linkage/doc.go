// Package linkage holds the linkage model of agent-docs/linkage-check-design.md —
// Linkage, Link, the sealed Joint set, Drive, LinkageLoop and Schedule — and
// the engines that pose a linkage and verify a drive or a box of joint
// values over the motion engine. B is the body reference type: the decad
// package instantiates every type here with its own *Body handle, so callers
// normally name them through decad.
package linkage
