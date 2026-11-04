SECTION_PROLOGUE(.resource_table,,)
{
 KEEP(*(.resource_table))
} GROUP_LINK_IN(RAMABLE_REGION)
