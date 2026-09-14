//! Fixed upstream provenance for the Basalt-compatible implementation.

/// Basalt upstream repository used as the semantic reference.
pub const BASALT_UPSTREAM_REPOSITORY: &str = "https://github.com/VladyslavUsenko/basalt";

/// Immutable upstream revision audited for this crate's first contracts.
pub const BASALT_UPSTREAM_SHA: &str = "0f3b2b52c807f70ff4e2973ce253c73329eea7bc";

/// License of the upstream Basalt repository.
pub const BASALT_UPSTREAM_LICENSE: &str = "BSD-3-Clause";

/// Header-only dependency repository used for image and optical-flow
/// numerical semantics.
pub const BASALT_HEADERS_UPSTREAM_REPOSITORY: &str =
    "https://gitlab.com/VladyslavUsenko/basalt-headers";

/// Immutable header dependency revision referenced by Basalt's vcpkg port at
/// [`BASALT_UPSTREAM_SHA`].
pub const BASALT_HEADERS_UPSTREAM_SHA: &str = "aa441ba3e51050c47ba1902537792a2e4db7e43d";

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn provenance_is_pinned() {
        assert_eq!(BASALT_UPSTREAM_SHA.len(), 40);
        assert_eq!(BASALT_UPSTREAM_LICENSE, "BSD-3-Clause");
        assert!(BASALT_UPSTREAM_REPOSITORY.ends_with("/basalt"));
        assert_eq!(BASALT_HEADERS_UPSTREAM_SHA.len(), 40);
        assert!(BASALT_HEADERS_UPSTREAM_REPOSITORY.ends_with("/basalt-headers"));
    }
}
