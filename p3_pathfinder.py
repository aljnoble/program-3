def find_path(src, dest, mesh):

    path = []
    visited = []

    for box in mesh['boxes']:

        if src[0] in range(box[0], box[1]):
            if src[1] in range(box[2], box[3]):
                src_box = box
                print 'Source in box ' + str(src_box)
        if dest[0] in range(box[0], box[1]):
            if dest[1] in range(box[2], box[3]):
                dest_box = box
                print 'Destination in box ' + str(dest_box)

    """Straight line on print-out"""
    path.append((src, dest))



    return path, visited

__author__ = 'Alec'
