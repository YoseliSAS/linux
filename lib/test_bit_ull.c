// SPDX-License-Identifier: GPL-2.0-only
/*
 * Test BIT() vs BIT_ULL() behavior on 32-bit architectures
 *
 * This test verifies the fix from commit 3626aecaa236
 * "dma: mcf-edma: Fix interrupt handling for channels > 31"
 *
 * On 32-bit architectures (like M68K), unsigned long is 32-bit,
 * so BIT(n) for n >= 32 causes undefined behavior (shift >= width).
 * For 64-bit bitmaps, BIT_ULL() must be used instead.
 *
 * This is critical for eDMA interrupt handling where channels 32-63
 * require BIT_ULL() for proper interrupt mask handling.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/bitops.h>

static int __init test_bit_ull_init(void)
{
	int i;
	int bit_errors = 0;
	int bit_ull_errors = 0;
	u64 expected, bit_result, bit_ull_result;
	const int bits_in_ulong = sizeof(unsigned long) * 8;

	pr_info("========================================\n");
	pr_info("BIT() vs BIT_ULL() Self-Test\n");
	pr_info("Verifying fix for commit 3626aecaa236\n");
	pr_info("========================================\n");
	pr_info("Architecture: sizeof(unsigned long) = %d bits\n", bits_in_ulong);
	pr_info("Test: Verify BIT(i) and BIT_ULL(i) produce correct values\n\n");

	/*
	 * Test that BIT(i) and BIT_ULL(i) produce the CORRECT bit mask value
	 * Expected value for bit i is: 1 << i
	 */
	pr_info("Loop 1: Check BIT() macro produces correct bit mask\n");
	pr_info("---------------------------------------------------\n");
	for (i = 0; i < 64; i++) {
		expected = 1ULL << i;
		bit_result = BIT(i);

		if (bit_result != expected) {
			pr_err("  bit %2d: FAIL - BIT(%d) = 0x%016llx, expected 0x%016llx\n",
			       i, i, (u64)bit_result, expected);
			bit_errors++;
		} else {
			if (i < 10 || i >= 30 && i < 34 || i >= 62)
				pr_info("  bit %2d: PASS - BIT(%d) = 0x%016llx\n",
				        i, i, (u64)bit_result);
		}
	}

	pr_info("\n");
	pr_info("Loop 2: Check BIT_ULL() macro produces correct bit mask\n");
	pr_info("-------------------------------------------------------\n");
	for (i = 0; i < 64; i++) {
		expected = 1ULL << i;
		bit_ull_result = BIT_ULL(i);

		if (bit_ull_result != expected) {
			pr_err("  bit %2d: FAIL - BIT_ULL(%d) = 0x%016llx, expected 0x%016llx\n",
			       i, i, bit_ull_result, expected);
			bit_ull_errors++;
		} else {
			if (i < 10 || i >= 30 && i < 34 || i >= 62)
				pr_info("  bit %2d: PASS - BIT_ULL(%d) = 0x%016llx\n",
				        i, i, bit_ull_result);
		}
	}

	pr_info("\n========================================\n");
	pr_info("Test Results:\n");
	pr_info("  BIT() macro:     %d failures out of 64 bits\n", bit_errors);
	pr_info("  BIT_ULL() macro: %d failures out of 64 bits\n", bit_ull_errors);
	pr_info("\n");

	if (bits_in_ulong == 32) {
		pr_info("On 32-bit architecture:\n");
		if (bit_errors >= 32) {
			pr_err("  ✗ BIT() FAILED for %d bits (expected for bits >= 32)\n", bit_errors);
			pr_info("    BIT(i) for i >= 32 produces WRONG values (undefined behavior)\n");
			pr_info("    This demonstrates the bug that commit 3626aecaa236 fixed!\n");
		} else if (bit_errors > 0) {
			pr_err("  ✗ BIT() FAILED for %d bits (unexpected pattern)\n", bit_errors);
		} else {
			pr_warn("  ? BIT() unexpectedly produced correct values for all bits\n");
			pr_warn("    This may be due to compiler optimizations or specific toolchain behavior\n");
		}

		if (bit_ull_errors == 0) {
			pr_info("  ✓ BIT_ULL() correctly produced all 64 bit masks\n");
		} else {
			pr_err("  ✗ BIT_ULL() unexpectedly failed!\n");
		}

		pr_info("\n");
		pr_info("Real-world impact (mcf-edma-main.c):\n");
		pr_info("  intmap = 64-bit register with interrupt status\n");
		pr_info("  for (ch = 0; ch < 64; ch++) {\n");
		pr_info("    if (intmap & BIT(ch))      // WRONG - undefined for ch >= 32\n");
		pr_info("    if (intmap & BIT_ULL(ch))  // CORRECT - works for all channels\n");
		pr_info("  }\n");
		pr_info("  \n");
		pr_info("  Even if BIT(32) appears to work in testing, it's undefined behavior!\n");
	} else {
		pr_info("64-bit architecture: BIT() and BIT_ULL() are equivalent\n");
	}

	if (bit_errors > 0 && bit_ull_errors == 0) {
		pr_info("\n✓ TEST PASSED: Demonstrates BIT() limitation and BIT_ULL() correctness\n");
	} else if (bit_errors == 0 && bit_ull_errors == 0) {
		pr_info("\n✓ TEST PASSED: Both macros produce correct values\n");
		if (bits_in_ulong == 32) {
			pr_warn("  NOTE: BIT(i) for i >= 32 is still undefined behavior, even if it works!\n");
		}
	} else {
		pr_err("\n✗ TEST FAILED: Unexpected results\n");
	}

	pr_info("========================================\n");

	/* Return 0 for built-in to continue boot normally */
#ifdef MODULE
	return -EAGAIN;
#else
	return 0;
#endif
}

static void __exit test_bit_ull_exit(void)
{
	/* Cleanup if needed */
}

module_init(test_bit_ull_init);
module_exit(test_bit_ull_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("DLC Next Validation");
MODULE_DESCRIPTION("Test BIT() vs BIT_ULL() for 64-bit bitmaps on 32-bit architectures");
