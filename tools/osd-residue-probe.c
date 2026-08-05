/*
 * osd-residue-probe — prove how THIS system's librockchip_mpp handles
 * KEY_OSD_DATA2 present -> absent across frames.
 *
 * The 2026 MPP tree resets the HAL's per-frame OSD pointers when the meta
 * key is absent; the Dec-2023 snapshot Radxa ships may keep the previous
 * frame's pointer in its task ring instead — which would re-blend a stale
 * overlay forever (the "lingering overlay" bug: side data detached, IDRs
 * fired, ghost still on stream). Encode 60 gray frames with OSD on frames
 * 5..15 only, then:
 *   mode absent   — no OSD meta after frame 15 (the current patch behavior)
 *   mode disabled — attach a single DISABLED region after frame 15 (the
 *                   candidate fix: the key exists every frame, osd_e is
 *                   written 0 explicitly)
 * Decode the output elsewhere; OSD visible after frame ~16 in mode
 * `absent` proves the stale-pointer latch on this library.
 *
 * Deliberately restricted to the old MPI surface (mpp_create/mpp_init,
 * MppEncCfg, encode_put_frame/get_packet) so it links against the 2023 lib.
 *
 * Usage: osd-residue-probe <absent|disabled> <h264|h265> <out.bin>
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <rockchip/rk_mpi.h>

#define W 1920
#define H 1080
#define FRAMES 60
#define OSD_FIRST 5
#define OSD_LAST 15

static MppEncOSDPlt plt;
static MppEncOSDData2 osd2;

static void fill_palette(void) {
    memset(&plt, 0, sizeof(plt));
    /* index 1: full-swing white, opaque — (alpha<<24)|(v<<16)|(u<<8)|y */
    plt.data[1].val = (255u << 24) | (128 << 16) | (128 << 8) | 235;
}

int main(int argc, char **argv) {
    if (argc != 4) {
        fprintf(stderr, "usage: %s <absent|disabled> <h264|h265> <out.bin>\n", argv[0]);
        return 2;
    }
    int disabled_mode = !strcmp(argv[1], "disabled");
    MppCodingType coding =
        !strcmp(argv[2], "h264") ? MPP_VIDEO_CodingAVC : MPP_VIDEO_CodingHEVC;
    FILE *out = fopen(argv[3], "wb");
    if (!out) { perror("out"); return 2; }

    MppCtx ctx = NULL;
    MppApi *mpi = NULL;
    if (mpp_create(&ctx, &mpi) || mpp_init(ctx, MPP_CTX_ENC, coding)) {
        fprintf(stderr, "mpp init failed\n");
        return 1;
    }

    MppEncCfg cfg = NULL;
    mpp_enc_cfg_init(&cfg);
    mpp_enc_cfg_set_s32(cfg, "prep:width", W);
    mpp_enc_cfg_set_s32(cfg, "prep:height", H);
    mpp_enc_cfg_set_s32(cfg, "prep:hor_stride", W);
    mpp_enc_cfg_set_s32(cfg, "prep:ver_stride", H);
    mpp_enc_cfg_set_s32(cfg, "prep:format", MPP_FMT_YUV420SP);
    mpp_enc_cfg_set_s32(cfg, "rc:mode", MPP_ENC_RC_MODE_FIXQP);
    mpp_enc_cfg_set_s32(cfg, "rc:qp_init", 15);
    mpp_enc_cfg_set_s32(cfg, "rc:qp_min", 15);
    mpp_enc_cfg_set_s32(cfg, "rc:qp_max", 15);
    mpp_enc_cfg_set_s32(cfg, "rc:qp_min_i", 15);
    mpp_enc_cfg_set_s32(cfg, "rc:qp_max_i", 15);
    mpp_enc_cfg_set_s32(cfg, "rc:fps_in_num", 60);
    mpp_enc_cfg_set_s32(cfg, "rc:fps_out_num", 60);
    mpp_enc_cfg_set_s32(cfg, "rc:gop", 60);
    if (mpi->control(ctx, MPP_ENC_SET_CFG, cfg)) {
        fprintf(stderr, "set cfg failed\n");
        return 1;
    }

    /* SPS/PPS/VPS up front so the output decodes standalone */
    {
        MppPacket hdr = NULL;
        char hdr_buf[4096];
        mpp_packet_init(&hdr, hdr_buf, sizeof(hdr_buf));
        mpp_packet_set_length(hdr, 0);
        if (!mpi->control(ctx, MPP_ENC_GET_HDR_SYNC, hdr))
            fwrite(mpp_packet_get_pos(hdr), 1, mpp_packet_get_length(hdr), out);
        mpp_packet_deinit(&hdr);
    }

    MppBufferGroup grp = NULL;
    mpp_buffer_group_get_internal(&grp, MPP_BUFFER_TYPE_DRM);
    MppBuffer frm_buf = NULL, pkt_buf = NULL, osd_buf = NULL;
    mpp_buffer_get(grp, &frm_buf, W * H * 3 / 2);
    mpp_buffer_get(grp, &pkt_buf, W * H);
    mpp_buffer_get(grp, &osd_buf, 4 * 2 * 256); /* 4x2 MBs */
    if (!frm_buf || !pkt_buf || !osd_buf) {
        fprintf(stderr, "buffer alloc failed\n");
        return 1;
    }
    /* gray frame + white-index OSD block, filled once */
    void *p = mpp_buffer_get_ptr(frm_buf);
    memset(p, 0x50, W * H);
    memset((char *) p + W * H, 0x80, W * H / 2);
    memset(mpp_buffer_get_ptr(osd_buf), 1, 4 * 2 * 256);
    fill_palette();

    for (int i = 0; i < FRAMES; i++) {
        MppFrame frame = NULL;
        mpp_frame_init(&frame);
        mpp_frame_set_width(frame, W);
        mpp_frame_set_height(frame, H);
        mpp_frame_set_hor_stride(frame, W);
        mpp_frame_set_ver_stride(frame, H);
        mpp_frame_set_fmt(frame, MPP_FMT_YUV420SP);
        mpp_frame_set_buffer(frame, frm_buf);
        mpp_frame_set_eos(frame, i == FRAMES - 1);

        int osd_on = (i >= OSD_FIRST && i <= OSD_LAST);
        if (osd_on || (disabled_mode && i > OSD_LAST)) {
            MppMeta meta = mpp_frame_get_meta(frame);
            if (osd_on) {
                MppEncOSDPltCfg plt_cfg;
                memset(&plt_cfg, 0, sizeof(plt_cfg));
                plt_cfg.change = MPP_ENC_OSD_PLT_CFG_CHANGE_ALL;
                plt_cfg.type = MPP_ENC_OSD_PLT_TYPE_USERDEF;
                plt_cfg.plt = &plt;
                mpi->control(ctx, MPP_ENC_SET_OSD_PLT_CFG, &plt_cfg);
            }
            memset(&osd2, 0, sizeof(osd2));
            osd2.num_region = 1;
            osd2.region[0].enable = osd_on ? 1 : 0;
            osd2.region[0].start_mb_x = 8;
            osd2.region[0].start_mb_y = 8;
            osd2.region[0].num_mb_x = osd_on ? 4 : 0;
            osd2.region[0].num_mb_y = osd_on ? 2 : 0;
            osd2.region[0].buf_offset = 0;
            osd2.region[0].buf = osd_on ? osd_buf : NULL;
            mpp_meta_set_ptr(meta, KEY_OSD_DATA2, &osd2);
        }

        if (mpi->encode_put_frame(ctx, frame)) {
            fprintf(stderr, "put_frame %d failed\n", i);
            return 1;
        }
        mpp_frame_deinit(&frame);

        MppPacket packet = NULL;
        if (mpi->encode_get_packet(ctx, &packet) || !packet) {
            fprintf(stderr, "get_packet %d failed\n", i);
            return 1;
        }
        fwrite(mpp_packet_get_pos(packet), 1, mpp_packet_get_length(packet), out);
        mpp_packet_deinit(&packet);
    }

    fclose(out);
    mpp_buffer_put(osd_buf);
    mpp_buffer_put(pkt_buf);
    mpp_buffer_put(frm_buf);
    mpp_buffer_group_put(grp);
    mpp_destroy(ctx);
    fprintf(stderr, "done: %s %s -> %s\n", argv[1], argv[2], argv[3]);
    return 0;
}
