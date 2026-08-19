#ifndef CONFIGURATIONISR_H
#define CONFIGURATIONISR_H

/* Host replacement for Configurations/ConfigurationIsr.h.
 *
 * The real file is the project-wide interrupt priority map (SRPNs for every
 * peripheral on every core). GnssM9N.c uses exactly one symbol from it, so
 * that is all this shim carries -- same rule as the other fakes: minimal
 * surface, not a copy.
 *
 * The value is mirrored from the real header on purpose. It never reaches
 * hardware here, but keeping it identical means a test that ever asserts on
 * the priority stays honest.
 */

#define ISR_PRIORITY_ASCLIN4_RX     105

#endif /* CONFIGURATIONISR_H */
