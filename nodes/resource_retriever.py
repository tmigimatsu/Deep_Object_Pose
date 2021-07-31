import re


def get_filename(url, use_protocol=False):
    package_path = re.match("package://dope/(.*)", url)[1]
    return "../" + package_path
