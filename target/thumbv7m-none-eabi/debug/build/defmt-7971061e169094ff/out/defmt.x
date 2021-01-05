/* exhaustively search for these symbols */
EXTERN(_defmt_acquire);
EXTERN(_defmt_release);
EXTERN(__defmt_default_timestamp);
PROVIDE(_defmt_timestamp = __defmt_default_timestamp);
PROVIDE(_defmt_panic = __defmt_default_panic);

SECTIONS
{
  /* `0` specifies the start address of this virtual (`(INFO)`) section */
  .defmt 0 (INFO) :
  {
    /* Format implementations for primitives like u8 */
    *(.defmt.prim.*);

    /* Everything user-defined */
    *(.defmt.*);

    /* 0.1 may contain special chars, so we quote the symbol name */
    /* Note that the quotes actually become part of the symbol name though! */
    "_defmt_version_ = 0.1" = 1;
  }
}

ASSERT(SIZEOF(.defmt) < 16384, ".defmt section cannot contain more than (1<<14) interned strings");
