/*===============KEYPAD_DRIVER===============
 * 8 pin from left to right: 0 -> 7
 *
 * INPUT_IO:	PA0 -> PA3 (PIN_0 -> PIN_3)
 * OUTPUT_IO:	PA4 -> PA7 (PIN_4 -> PIN_7)
 * ==========================================
 */
#include "keypad_driver.h"

static void check_trang_thai_phim(bool *phim_co_nhan, uint8_t *gia_tri_phim_nhan, uint8_t *to_hop_phim);
static void doc_gia_tri_phim(uint8_t vi_tri_cot, uint32_t trang_thai_hang, uint8_t *gia_tri_phim_nhan, uint8_t *to_hop_phim);

uint8_t keypad_4x4_return_gia_tri_phim_nhan(void)
{
	uint8_t gia_tri_phim_nhan;
	uint8_t to_hop_phim [16] = {'1', '4', '7', '*', '2', '5', '8', '0', '3', '6', '9', '#', 'A', 'B', 'C', 'D'};
	bool phim_co_nhan = true;

	for (int8_t nhan_count = 1; nhan_count <= 50; nhan_count++)
	{
		check_trang_thai_phim(&phim_co_nhan, &gia_tri_phim_nhan, to_hop_phim);

		if (phim_co_nhan == false) {
			nhan_count = 0;
			continue;
		}
	}

	for (int8_t nha_count = 1; nha_count <= 50; nha_count++)
	{
		check_trang_thai_phim(&phim_co_nhan, &gia_tri_phim_nhan, to_hop_phim);

		if (phim_co_nhan == true) {
			nha_count = 0;
			continue;
		}
	}

	return gia_tri_phim_nhan;
}

static void check_trang_thai_phim(bool *phim_co_nhan, uint8_t *gia_tri_phim_nhan, uint8_t *to_hop_phim)
{
	uint8_t vi_tri_cot = 0, gia_tri_xuat_cot = 16;

	for (; vi_tri_cot <= 3; vi_tri_cot++, gia_tri_xuat_cot <<= 1)
	{
		KEYPAD_OUT_REGISTER = gia_tri_xuat_cot;

		uint32_t trang_thai_hang = KEYPAD_IN_REGISTER;
		trang_thai_hang &= 0x000F;

		if (trang_thai_hang != 0)
		{
			*phim_co_nhan = true;
			doc_gia_tri_phim(vi_tri_cot, trang_thai_hang, gia_tri_phim_nhan, to_hop_phim);
			return;
		}

		else
			*phim_co_nhan = false;
	}
}

static void doc_gia_tri_phim(uint8_t vi_tri_cot, uint32_t trang_thai_hang, uint8_t *gia_tri_phim_nhan, uint8_t *to_hop_phim)
{
	uint8_t vi_tri_hang = 0, vi_tri_nhan = 1;

	for (; vi_tri_hang <= 12; vi_tri_hang += 4, vi_tri_nhan <<= 1)
	{
		if ((trang_thai_hang & vi_tri_nhan) != 0)

			*gia_tri_phim_nhan = to_hop_phim [vi_tri_cot + vi_tri_hang];
	}
}
